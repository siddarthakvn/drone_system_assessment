import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from drone_system.log_utils import log_event


class FollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('follower_node')

        self.declare_parameter('follow_offset_x_m', -6.0)
        self.declare_parameter('follow_offset_y_m', 0.0)
        self.declare_parameter('follow_altitude_m', 20.0)
        self.declare_parameter('car_position_timeout_s', 0.2)
        self.declare_parameter('max_position_jump_m', 5.0)
        self.declare_parameter('frame_id', 'map')

        self.follow_offset_x_m = self.get_parameter('follow_offset_x_m').value
        self.follow_offset_y_m = self.get_parameter('follow_offset_y_m').value
        self.follow_altitude_m = self.get_parameter('follow_altitude_m').value
        self.car_position_timeout_s = self.get_parameter('car_position_timeout_s').value
        self.max_position_jump_m = self.get_parameter('max_position_jump_m').value
        self.frame_id = self.get_parameter('frame_id').value

        self.last_car_pose: Optional[PoseStamped] = None
        self.last_valid_time = None
        self.last_waypoint: Optional[PoseStamped] = None
        self.timeout_active = False

        self.waypoint_publisher = self.create_publisher(PoseStamped, '/drone/waypoint', 10)
        self.subscription = self.create_subscription(PoseStamped, '/car/position', self.handle_car_pose, 10)
        self.timer = self.create_timer(0.05, self.check_for_timeout)

        log_event(
            self,
            'info',
            'follower_node',
            'started',
            'Follower node is waiting for /car/position and will publish /drone/waypoint.',
        )

    def handle_car_pose(self, message: PoseStamped) -> None:
        if self.last_car_pose is not None:
            dx = message.pose.position.x - self.last_car_pose.pose.position.x
            dy = message.pose.position.y - self.last_car_pose.pose.position.y
            dz = message.pose.position.z - self.last_car_pose.pose.position.z
            step_distance = math.sqrt(dx * dx + dy * dy + dz * dz)

            if step_distance > self.max_position_jump_m:
                log_event(
                    self,
                    'warning',
                    'follower_node',
                    'position_jump_discarded',
                    f'Discarded car position jump of {step_distance:.2f} m; holding last valid target.',
                )
                return

        self.last_car_pose = message
        self.last_valid_time = self.get_clock().now()
        self.timeout_active = False
        self.publish_offset_waypoint(message)

    def publish_offset_waypoint(self, car_pose: PoseStamped) -> None:
        waypoint = PoseStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = self.frame_id
        waypoint.pose.position.x = car_pose.pose.position.x + self.follow_offset_x_m
        waypoint.pose.position.y = car_pose.pose.position.y + self.follow_offset_y_m
        waypoint.pose.position.z = self.follow_altitude_m
        waypoint.pose.orientation.w = 1.0

        self.last_waypoint = waypoint
        self.waypoint_publisher.publish(waypoint)

    def check_for_timeout(self) -> None:
        if self.last_valid_time is None:
            return

        gap_s = (self.get_clock().now() - self.last_valid_time).nanoseconds / 1e9
        if gap_s <= self.car_position_timeout_s or self.timeout_active:
            return

        self.timeout_active = True

        if self.last_waypoint is not None:
            self.last_waypoint.header.stamp = self.get_clock().now().to_msg()
            self.waypoint_publisher.publish(self.last_waypoint)

        log_event(
            self,
            'error',
            'follower_node',
            'car_position_timeout',
            f'No /car/position received for {gap_s:.3f} s; commanding hover at the last valid waypoint.',
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
