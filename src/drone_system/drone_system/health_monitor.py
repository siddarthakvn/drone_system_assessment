from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from drone_system.log_utils import log_event


class HealthMonitor(Node):
    def __init__(self) -> None:
        super().__init__('health_monitor')

        self.declare_parameter('rtf_warning_threshold', 0.8)
        self.declare_parameter('rtf_warning_interval_s', 5.0)

        self.rtf_warning_threshold = float(self.get_parameter('rtf_warning_threshold').value)
        self.rtf_warning_interval_s = float(self.get_parameter('rtf_warning_interval_s').value)

        self.low_rtf_active = False
        self.last_warning_time: Optional[object] = None
        self.last_rtf = None

        self.subscription = self.create_subscription(Float32, '/simulation/rtf', self.handle_rtf, 10)

        log_event(
            self,
            'info',
            'health_monitor',
            'started',
            'Health monitor subscribed to /simulation/rtf for real-time factor warnings.',
        )

    def handle_rtf(self, message: Float32) -> None:
        now = self.get_clock().now()
        self.last_rtf = float(message.data)

        if self.last_rtf < self.rtf_warning_threshold:
            should_warn = self.last_warning_time is None
            if self.last_warning_time is not None:
                delta_s = (now - self.last_warning_time).nanoseconds / 1e9
                should_warn = delta_s >= self.rtf_warning_interval_s

            self.low_rtf_active = True
            if should_warn:
                self.last_warning_time = now
                log_event(
                    self,
                    'warning',
                    'health_monitor',
                    'low_rtf',
                    f'Gazebo real-time factor is {self.last_rtf:.2f}, below the {self.rtf_warning_threshold:.2f} threshold.',
                )
            return

        if self.low_rtf_active:
            self.low_rtf_active = False
            self.last_warning_time = None
            log_event(
                self,
                'info',
                'health_monitor',
                'rtf_recovered',
                f'Gazebo real-time factor recovered to {self.last_rtf:.2f}.',
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
