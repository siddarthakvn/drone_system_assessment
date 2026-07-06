import asyncio
import math
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw
from rclpy.node import Node

from drone_system.log_utils import log_event


@dataclass
class LocalPose:
    north_m: float
    east_m: float
    up_m: float
    heading_deg: float


class PX4Manager(Node):
    def __init__(self) -> None:
        super().__init__('px4_manager')

        self.declare_parameter('system_address', 'udpin://0.0.0.0:14540')
        self.declare_parameter('arm_retry_limit', 3)
        self.declare_parameter('arm_retry_period_s', 1.0)
        self.declare_parameter('takeoff_altitude_m', 20.0)
        self.declare_parameter('control_period_s', 0.05)
        self.declare_parameter('takeoff_reached_tolerance_m', 0.75)
        self.declare_parameter('pre_arm_delay_s', 2.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('default_yaw_deg', 0.0)

        self.system_address = self.get_parameter('system_address').value
        self.arm_retry_limit = int(self.get_parameter('arm_retry_limit').value)
        self.arm_retry_period_s = float(self.get_parameter('arm_retry_period_s').value)
        self.takeoff_altitude_m = float(self.get_parameter('takeoff_altitude_m').value)
        self.control_period_s = float(self.get_parameter('control_period_s').value)
        self.takeoff_reached_tolerance_m = float(self.get_parameter('takeoff_reached_tolerance_m').value)
        self.pre_arm_delay_s = float(self.get_parameter('pre_arm_delay_s').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.default_yaw_deg = float(self.get_parameter('default_yaw_deg').value)

        self.pose_publisher = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.waypoint_subscriber = self.create_subscription(
            PoseStamped,
            '/drone/waypoint',
            self.handle_waypoint,
            10,
        )
        self.pose_timer = self.create_timer(0.05, self.publish_pose)

        self.current_waypoint: Optional[PoseStamped] = None
        self.current_pose: Optional[LocalPose] = None
        self.last_command: Optional[PositionNedYaw] = None
        self.phase = 'connecting'
        self.shutdown_requested = False
        self.offboard_started = False
        self.ros_shutdown_requested = False

        self.state_lock = threading.Lock()
        self.log_lock = threading.Lock()
        self.pending_logs = []
        self.async_loop = asyncio.new_event_loop()
        self.worker_thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.worker_thread.start()

        log_event(
            self,
            'info',
            'px4_manager',
            'started',
            f'PX4 manager started and is waiting to connect on {self.system_address}.',
        )

    def handle_waypoint(self, waypoint: PoseStamped) -> None:
        with self.state_lock:
            self.current_waypoint = waypoint

    def publish_pose(self) -> None:
        self.flush_pending_logs()

        if self.ros_shutdown_requested:
            self.ros_shutdown_requested = False
            rclpy.shutdown()
            return

        with self.state_lock:
            pose = self.current_pose

        if pose is None:
            return

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        # Publish ROS world pose in ENU/map coordinates.
        message.pose.position.x = pose.east_m
        message.pose.position.y = pose.north_m
        message.pose.position.z = pose.up_m
        yaw_rad = math.radians(pose.heading_deg)
        message.pose.orientation.z = math.sin(yaw_rad / 2.0)
        message.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self.pose_publisher.publish(message)

    def request_ros_shutdown(self) -> None:
        self.ros_shutdown_requested = True

    def queue_log(self, severity: str, event_type: str, description: str) -> None:
        with self.log_lock:
            self.pending_logs.append((severity, event_type, description))

    def flush_pending_logs(self) -> None:
        with self.log_lock:
            pending = list(self.pending_logs)
            self.pending_logs.clear()

        for severity, event_type, description in pending:
            log_event(self, severity, 'px4_manager', event_type, description)

    def _run_event_loop(self) -> None:
        asyncio.set_event_loop(self.async_loop)
        self.async_loop.run_until_complete(self.run_px4_manager())

    async def run_px4_manager(self) -> None:
        drone = System()
        telemetry_task = None
        heading_task = None

        try:
            await drone.connect(system_address=self.system_address)
            self.queue_log('info', 'connecting', f'Connecting to PX4 on {self.system_address}.')

            async for state in drone.core.connection_state():
                if state.is_connected:
                    self.phase = 'arming'
                    self.queue_log('info', 'connected', 'Connected to PX4 SITL over MAVSDK.')
                    break

            telemetry_task = asyncio.create_task(self.telemetry_loop(drone))
            heading_task = asyncio.create_task(self.heading_loop(drone))

            await drone.telemetry.set_rate_position_velocity_ned(20.0)

            await self.wait_for_local_position()
            if self.pre_arm_delay_s > 0.0:
                self.queue_log(
                    'info',
                    'pre_arm_delay',
                    f'Waiting {self.pre_arm_delay_s:.1f} s before arming so PX4 can finish pre-arm checks.',
                )
                await asyncio.sleep(self.pre_arm_delay_s)
            await self.arm_with_retries(drone)
            await self.start_offboard(drone)
            await self.command_loop(drone)
        except asyncio.CancelledError:
            pass
        except Exception as error:  # pragma: no cover - defensive runtime path
            self.queue_log('error', 'runtime_failure', f'PX4 manager stopped due to unexpected error: {error}')
        finally:
            self.shutdown_requested = True
            if telemetry_task is not None:
                telemetry_task.cancel()
            if heading_task is not None:
                heading_task.cancel()

            if self.offboard_started:
                try:
                    await drone.offboard.stop()
                except Exception:
                    pass

    async def wait_for_local_position(self) -> None:
        while not self.shutdown_requested:
            with self.state_lock:
                pose_available = self.current_pose is not None

            if pose_available:
                self.queue_log('info', 'local_position_ready', 'PX4 local position estimate is available.')
                return

            await asyncio.sleep(0.1)

    async def arm_with_retries(self, drone: System) -> None:
        for attempt in range(1, self.arm_retry_limit + 1):
            try:
                await drone.action.arm()
                self.phase = 'takeoff'
                self.queue_log('info', 'armed', f'Arming succeeded on attempt {attempt}; climbing to {self.takeoff_altitude_m:.1f} m.')
                return
            except ActionError as error:
                severity = 'warning' if attempt < self.arm_retry_limit else 'error'
                event_type = 'arm_retry' if attempt < self.arm_retry_limit else 'arm_failed_shutdown'
                description = (
                    f'Arming attempt {attempt}/{self.arm_retry_limit} failed with result '
                    f'{error._result.result}; retrying.'
                )
                if attempt == self.arm_retry_limit:
                    description = (
                        f'Arming failed after {self.arm_retry_limit} attempts with result '
                        f'{error._result.result}; shutting down cleanly.'
                    )

                self.queue_log(severity, event_type, description)
                if attempt == self.arm_retry_limit:
                    self.request_ros_shutdown()
                    raise

                await asyncio.sleep(self.arm_retry_period_s)

    async def start_offboard(self, drone: System) -> None:
        initial_setpoint = self.compute_hold_setpoint()
        await drone.offboard.set_position_ned(initial_setpoint)

        try:
            await drone.offboard.start()
        except OffboardError as error:
            self.queue_log(
                'error',
                'offboard_start_failed',
                f'Offboard start failed with result {error._result.result}; shutting down cleanly.',
            )
            self.request_ros_shutdown()
            raise

        self.offboard_started = True
        self.queue_log('info', 'offboard_started', 'PX4 offboard control started successfully.')

    async def command_loop(self, drone: System) -> None:
        while not self.shutdown_requested:
            target = self.compute_target_setpoint()
            self.last_command = target
            await drone.offboard.set_position_ned(target)
            await asyncio.sleep(self.control_period_s)

    def compute_hold_setpoint(self) -> PositionNedYaw:
        with self.state_lock:
            pose = self.current_pose

        if pose is None:
            return PositionNedYaw(0.0, 0.0, 0.0, self.default_yaw_deg)

        return PositionNedYaw(
            pose.north_m,
            pose.east_m,
            -pose.up_m,
            pose.heading_deg if pose.heading_deg == pose.heading_deg else self.default_yaw_deg,
        )

    def compute_target_setpoint(self) -> PositionNedYaw:
        with self.state_lock:
            pose = self.current_pose
            waypoint = self.current_waypoint

        if pose is None:
            return PositionNedYaw(0.0, 0.0, -self.takeoff_altitude_m, self.default_yaw_deg)

        if self.phase == 'takeoff':
            if pose.up_m >= self.takeoff_altitude_m - self.takeoff_reached_tolerance_m:
                self.phase = 'follow'
                self.queue_log('info', 'takeoff_complete', 'Takeoff complete; switching to follow mode.')

            return PositionNedYaw(
                pose.north_m,
                pose.east_m,
                -self.takeoff_altitude_m,
                self.default_yaw_deg,
            )

        if waypoint is None:
            return PositionNedYaw(
                pose.north_m,
                pose.east_m,
                -pose.up_m,
                self.default_yaw_deg,
            )

        # Convert ROS map/ENU waypoints into PX4 local NED setpoints.
        return PositionNedYaw(
            waypoint.pose.position.y,
            waypoint.pose.position.x,
            -waypoint.pose.position.z,
            self.default_yaw_deg,
        )

    async def telemetry_loop(self, drone: System) -> None:
        async for position_velocity in drone.telemetry.position_velocity_ned():
            heading_deg = self.default_yaw_deg
            with self.state_lock:
                existing_pose = self.current_pose
                if existing_pose is not None:
                    heading_deg = existing_pose.heading_deg

                self.current_pose = LocalPose(
                    north_m=position_velocity.position.north_m,
                    east_m=position_velocity.position.east_m,
                    up_m=-position_velocity.position.down_m,
                    heading_deg=heading_deg,
                )

            if self.shutdown_requested:
                return

    async def heading_loop(self, drone: System) -> None:
        async for heading in drone.telemetry.heading():
            with self.state_lock:
                if self.current_pose is not None:
                    self.current_pose.heading_deg = heading.heading_deg

            if self.shutdown_requested:
                return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PX4Manager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_requested = True
        if node.async_loop.is_running():
            node.async_loop.call_soon_threadsafe(lambda: None)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
