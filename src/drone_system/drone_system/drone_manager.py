from copy import deepcopy
from typing import Optional

import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from drone_system.log_utils import log_event


class DroneManager(Node):
    def __init__(self) -> None:
        super().__init__('drone_manager')

        self.declare_parameter('arm_failures_before_success', 0)
        self.declare_parameter('arm_retry_limit', 3)
        self.declare_parameter('arm_retry_period_s', 1.0)
        self.declare_parameter('takeoff_altitude_m', 20.0)
        self.declare_parameter('climb_rate_mps', 2.0)
        self.declare_parameter('tracking_gain', 0.15)
        self.declare_parameter('control_period_s', 0.05)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('model_name', 'drone')
        self.declare_parameter('spawn_altitude_m', 0.25)

        self.arm_failures_before_success = int(self.get_parameter('arm_failures_before_success').value)
        self.arm_retry_limit = int(self.get_parameter('arm_retry_limit').value)
        self.arm_retry_period_s = float(self.get_parameter('arm_retry_period_s').value)
        self.takeoff_altitude_m = float(self.get_parameter('takeoff_altitude_m').value)
        self.climb_rate_mps = float(self.get_parameter('climb_rate_mps').value)
        self.tracking_gain = float(self.get_parameter('tracking_gain').value)
        self.control_period_s = float(self.get_parameter('control_period_s').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.model_name = self.get_parameter('model_name').value
        self.spawn_altitude_m = float(self.get_parameter('spawn_altitude_m').value)

        self.arm_attempts = 0
        self.phase = 'arming'
        self.current_waypoint: Optional[PoseStamped] = None
        self.current_pose: Optional[PoseStamped] = None
        self.model_seen = False
        self.service_ready_logged = False

        self.pose_publisher = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.subscription = self.create_subscription(PoseStamped, '/drone/waypoint', self.handle_waypoint, 10)
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.handle_model_states,
            10,
        )
        self.model_states_sub_alt = self.create_subscription(
            ModelStates,
            '/model_states',
            self.handle_model_states,
            10,
        )
        self.state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.state_client_alt = self.create_client(SetEntityState, '/set_entity_state')
        self.control_timer = self.create_timer(self.control_period_s, self.advance_state)
        self.arm_timer = self.create_timer(self.arm_retry_period_s, self.try_arm)

        log_event(
            self,
            'info',
            'drone_manager',
            'started',
            f'Drone manager will command Gazebo model "{self.model_name}" and is attempting to arm.',
        )

    def handle_waypoint(self, waypoint: PoseStamped) -> None:
        self.current_waypoint = waypoint

    def handle_model_states(self, message: ModelStates) -> None:
        if self.model_name not in message.name:
            return

        index = message.name.index(self.model_name)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose = deepcopy(message.pose[index])
        if pose_msg.pose.position.z < self.spawn_altitude_m:
            pose_msg.pose.position.z = self.spawn_altitude_m

        self.current_pose = pose_msg
        self.pose_publisher.publish(pose_msg)

        if not self.model_seen:
            self.model_seen = True
            log_event(
                self,
                'info',
                'drone_manager',
                'model_ready',
                f'Gazebo model "{self.model_name}" is present and ready for takeoff.',
            )

    def try_arm(self) -> None:
        if self.phase != 'arming':
            return

        self.arm_attempts += 1
        if self.arm_attempts <= self.arm_failures_before_success:
            log_event(
                self,
                'warning',
                'drone_manager',
                'arm_retry',
                f'Arming attempt {self.arm_attempts}/{self.arm_retry_limit} failed; retrying.',
            )
            if self.arm_attempts >= self.arm_retry_limit:
                log_event(
                    self,
                    'error',
                    'drone_manager',
                    'arm_failed_shutdown',
                    'Arming failed after configured retries; shutting down cleanly.',
                )
                rclpy.shutdown()
            return

        self.phase = 'takeoff'
        self.arm_timer.cancel()
        log_event(
            self,
            'info',
            'drone_manager',
            'armed',
            f'Arming succeeded on attempt {self.arm_attempts}; climbing to {self.takeoff_altitude_m:.1f} m.',
        )

    def advance_state(self) -> None:
        state_client = self.get_state_client()
        if state_client is None:
            if not self.service_ready_logged:
                log_event(
                    self,
                    'info',
                    'drone_manager',
                    'waiting_for_gazebo',
                    'Waiting for /gazebo/set_entity_state or /set_entity_state before commanding the drone.',
                )
                self.service_ready_logged = True
            return

        if self.current_pose is None:
            return

        if self.phase == 'arming':
            self.pose_publisher.publish(self.current_pose)
            return

        target = deepcopy(self.current_pose)
        if self.phase == 'takeoff':
            climb_step = self.climb_rate_mps * self.control_period_s
            target.pose.position.z = min(self.takeoff_altitude_m, self.current_pose.pose.position.z + climb_step)
            if target.pose.position.z >= self.takeoff_altitude_m:
                self.phase = 'follow'
                log_event(
                    self,
                    'info',
                    'drone_manager',
                    'takeoff_complete',
                    'Takeoff complete; switching to follow mode.',
                )
        elif self.current_waypoint is not None:
            target.pose.position.x = self.current_pose.pose.position.x + (
                self.current_waypoint.pose.position.x - self.current_pose.pose.position.x
            ) * self.tracking_gain
            target.pose.position.y = self.current_pose.pose.position.y + (
                self.current_waypoint.pose.position.y - self.current_pose.pose.position.y
            ) * self.tracking_gain
            target.pose.position.z = self.current_pose.pose.position.z + (
                self.current_waypoint.pose.position.z - self.current_pose.pose.position.z
            ) * self.tracking_gain

        self.command_pose(target, state_client)

    def get_state_client(self):
        if self.state_client.service_is_ready():
            return self.state_client
        if self.state_client_alt.service_is_ready():
            return self.state_client_alt
        return None

    def command_pose(self, target_pose: PoseStamped, state_client) -> None:
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.model_name
        request.state.reference_frame = 'world'
        request.state.pose = deepcopy(target_pose.pose)
        state_client.call_async(request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
