import math
import time

import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32

from drone_system.log_utils import log_event


class CarSimulator(Node):
    def __init__(self) -> None:
        super().__init__('car_simulator')

        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('path_radius_m', 12.0)
        self.declare_parameter('path_center_x_m', 0.0)
        self.declare_parameter('path_center_y_m', 0.0)
        self.declare_parameter('path_angular_speed_radps', 0.2)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('model_name', 'car')
        self.declare_parameter('model_z_m', 0.05)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.path_radius_m = float(self.get_parameter('path_radius_m').value)
        self.path_center_x_m = float(self.get_parameter('path_center_x_m').value)
        self.path_center_y_m = float(self.get_parameter('path_center_y_m').value)
        self.path_angular_speed_radps = float(self.get_parameter('path_angular_speed_radps').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.model_name = self.get_parameter('model_name').value
        self.model_z_m = float(self.get_parameter('model_z_m').value)

        self.pose_publisher = self.create_publisher(PoseStamped, '/car/position', 10)
        self.rtf_publisher = self.create_publisher(Float32, '/simulation/rtf', 10)
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
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.handle_clock,
            qos_profile_sensor_data,
        )
        self.state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.state_client_alt = self.create_client(SetEntityState, '/set_entity_state')
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_state)

        self.start_time = time.monotonic()
        self.last_wall_clock_s = None
        self.last_sim_clock_s = None
        self.model_seen = False
        self.service_ready_logged = False

        log_event(
            self,
            'info',
            'car_simulator',
            'started',
            f'Driving Gazebo model "{self.model_name}" and publishing /car/position at {self.publish_rate_hz:.1f} Hz.',
        )

    def publish_state(self) -> None:
        state_client = self.get_state_client()
        if state_client is None:
            if not self.service_ready_logged:
                log_event(
                    self,
                    'info',
                    'car_simulator',
                    'waiting_for_gazebo',
                    'Waiting for /gazebo/set_entity_state or /set_entity_state before commanding the car.',
                )
                self.service_ready_logged = True
            return

        elapsed = time.monotonic() - self.start_time
        theta = self.path_angular_speed_radps * elapsed
        yaw = theta + (math.pi / 2.0)

        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.model_name
        request.state.reference_frame = 'world'
        request.state.pose.position.x = self.path_center_x_m + self.path_radius_m * math.cos(theta)
        request.state.pose.position.y = self.path_center_y_m + self.path_radius_m * math.sin(theta)
        request.state.pose.position.z = self.model_z_m
        request.state.pose.orientation.z = math.sin(yaw / 2.0)
        request.state.pose.orientation.w = math.cos(yaw / 2.0)

        state_client.call_async(request)

    def get_state_client(self):
        if self.state_client.service_is_ready():
            return self.state_client
        if self.state_client_alt.service_is_ready():
            return self.state_client_alt
        return None

    def handle_model_states(self, message: ModelStates) -> None:
        if self.model_name not in message.name:
            return

        if not self.model_seen:
            self.model_seen = True
            log_event(
                self,
                'info',
                'car_simulator',
                'model_ready',
                f'Gazebo model "{self.model_name}" is present and publishing measured pose.',
            )

        index = message.name.index(self.model_name)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose = message.pose[index]
        self.pose_publisher.publish(pose)

    def handle_clock(self, message: Clock) -> None:
        current_wall_s = time.monotonic()
        current_sim_s = float(message.clock.sec) + float(message.clock.nanosec) / 1e9

        if self.last_wall_clock_s is not None and current_wall_s > self.last_wall_clock_s:
            rtf = (current_sim_s - self.last_sim_clock_s) / (current_wall_s - self.last_wall_clock_s)
            published_rtf = Float32()
            published_rtf.data = max(0.0, float(rtf))
            self.rtf_publisher.publish(published_rtf)

        self.last_wall_clock_s = current_wall_s
        self.last_sim_clock_s = current_sim_s


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
