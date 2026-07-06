import csv
import math
import re
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import Float32

from drone_system.log_utils import format_event, log_event


STRUCTURED_EVENT_RE = re.compile(r'^\d{4}-\d{2}-\d{2}T[^ ]+ [A-Z]+ \S+ \S+ .+')


class TelemetryRecorder(Node):
    def __init__(self) -> None:
        super().__init__('telemetry_recorder')

        self.declare_parameter('output_root', 'artifacts')
        self.declare_parameter('run_name', '')
        self.declare_parameter('sample_rate_hz', 10.0)

        output_root = Path(self.get_parameter('output_root').value)
        run_name = self.get_parameter('run_name').value.strip()
        self.sample_rate_hz = float(self.get_parameter('sample_rate_hz').value)

        if not run_name:
            run_name = datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%SZ')

        self.run_dir = output_root / run_name
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.run_dir / 'telemetry.csv'

        self.csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
        self.events_path = self.run_dir / 'events.log'
        self.events_file = self.events_path.open('w', encoding='utf-8')
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                'time_s',
                'car_x',
                'car_y',
                'car_z',
                'drone_x',
                'drone_y',
                'drone_z',
                'waypoint_x',
                'waypoint_y',
                'waypoint_z',
                'car_message_rate_hz',
                'drone_message_rate_hz',
                'rtf',
            ],
        )
        self.writer.writeheader()
        self.csv_file.flush()

        self.start_time = time.monotonic()
        self.samples_written = 0
        self.closed = False

        self.latest_car_pose: Optional[PoseStamped] = None
        self.latest_drone_pose: Optional[PoseStamped] = None
        self.latest_waypoint: Optional[PoseStamped] = None
        self.latest_rtf = math.nan

        self.car_arrivals = deque(maxlen=200)
        self.drone_arrivals = deque(maxlen=200)

        self.create_subscription(PoseStamped, '/car/position', self.handle_car_pose, 10)
        self.create_subscription(PoseStamped, '/drone/pose', self.handle_drone_pose, 10)
        self.create_subscription(PoseStamped, '/drone/waypoint', self.handle_waypoint, 10)
        self.create_subscription(Float32, '/simulation/rtf', self.handle_rtf, 10)
        self.create_subscription(Log, '/rosout', self.handle_rosout, 100)
        self.timer = self.create_timer(1.0 / self.sample_rate_hz, self.record_sample)

        log_event(
            self,
            'info',
            'telemetry_recorder',
            'started',
            f'Writing telemetry samples to {self.csv_path} at {self.sample_rate_hz:.1f} Hz.',
        )

    def handle_car_pose(self, message: PoseStamped) -> None:
        self.latest_car_pose = message
        self.car_arrivals.append(time.monotonic())

    def handle_drone_pose(self, message: PoseStamped) -> None:
        self.latest_drone_pose = message
        self.drone_arrivals.append(time.monotonic())

    def handle_waypoint(self, message: PoseStamped) -> None:
        self.latest_waypoint = message

    def handle_rtf(self, message: Float32) -> None:
        self.latest_rtf = float(message.data)

    def handle_rosout(self, message: Log) -> None:
        if self.closed:
            return
        if not STRUCTURED_EVENT_RE.match(message.msg):
            return
        self.events_file.write(message.msg + '\n')
        self.events_file.flush()

    def compute_message_rate_hz(self, arrivals: deque) -> float:
        if len(arrivals) < 2:
            return 0.0

        window_s = arrivals[-1] - arrivals[0]
        if window_s <= 0.0:
            return 0.0

        return float(len(arrivals) - 1) / window_s

    def record_sample(self) -> None:
        if self.latest_car_pose is None or self.latest_drone_pose is None:
            return

        waypoint = self.latest_waypoint
        row = {
            'time_s': time.monotonic() - self.start_time,
            'car_x': self.latest_car_pose.pose.position.x,
            'car_y': self.latest_car_pose.pose.position.y,
            'car_z': self.latest_car_pose.pose.position.z,
            'drone_x': self.latest_drone_pose.pose.position.x,
            'drone_y': self.latest_drone_pose.pose.position.y,
            'drone_z': self.latest_drone_pose.pose.position.z,
            'waypoint_x': waypoint.pose.position.x if waypoint is not None else math.nan,
            'waypoint_y': waypoint.pose.position.y if waypoint is not None else math.nan,
            'waypoint_z': waypoint.pose.position.z if waypoint is not None else math.nan,
            'car_message_rate_hz': self.compute_message_rate_hz(self.car_arrivals),
            'drone_message_rate_hz': self.compute_message_rate_hz(self.drone_arrivals),
            'rtf': self.latest_rtf,
        }
        self.writer.writerow(row)
        self.csv_file.flush()
        self.samples_written += 1

    def close(self) -> None:
        if self.closed:
            return

        self.closed = True
        completion_message = format_event(
            'info',
            'telemetry_recorder',
            'completed',
            f'Wrote {self.samples_written} telemetry samples to {self.csv_path}.',
        )
        self.events_file.write(completion_message + '\n')
        self.events_file.flush()
        self.csv_file.flush()
        self.csv_file.close()
        self.events_file.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
