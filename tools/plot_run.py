#!/usr/bin/env python3
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


REQUIRED_COLUMNS = {
    'time_s',
    'car_x',
    'car_y',
    'drone_x',
    'drone_y',
    'drone_z',
    'car_message_rate_hz',
    'rtf',
}


def plot_run(csv_path: Path, output_dir: Path) -> int:
    data = pd.read_csv(csv_path)
    missing = sorted(REQUIRED_COLUMNS - set(data.columns))
    if missing:
        print(f'missing required columns: {", ".join(missing)}', file=sys.stderr)
        return 1
    if len(data.index) < 2:
        print('telemetry CSV needs at least 2 rows to produce plots', file=sys.stderr)
        return 1

    output_dir.mkdir(parents=True, exist_ok=True)
    time_values = data['time_s'].to_numpy()
    car_x = data['car_x'].to_numpy()
    car_y = data['car_y'].to_numpy()
    drone_x = data['drone_x'].to_numpy()
    drone_y = data['drone_y'].to_numpy()
    drone_z = data['drone_z'].to_numpy()
    car_rate = data['car_message_rate_hz'].to_numpy()
    rtf = data['rtf'].to_numpy()

    figure = plt.figure()
    plt.plot(car_x, car_y, label='car')
    plt.plot(drone_x, drone_y, label='drone')
    if {'waypoint_x', 'waypoint_y'}.issubset(data.columns):
        plt.plot(data['waypoint_x'].to_numpy(), data['waypoint_y'].to_numpy(), '--', label='waypoint')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Drone XY Path vs Car XY Path')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    figure.tight_layout()
    figure.savefig(output_dir / 'xy_paths.png', bbox_inches='tight')
    plt.close(figure)

    figure = plt.figure()
    plt.plot(time_values, car_rate, label='car')
    if 'drone_message_rate_hz' in data.columns:
        plt.plot(time_values, data['drone_message_rate_hz'].to_numpy(), label='drone')
    plt.xlabel('time (s)')
    plt.ylabel('message rate (Hz)')
    plt.title('Topic Message Rate Over Time')
    plt.legend()
    plt.grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(output_dir / 'message_rate.png', bbox_inches='tight')
    plt.close(figure)

    figure = plt.figure()
    plt.plot(time_values, rtf)
    plt.xlabel('time (s)')
    plt.ylabel('real-time factor')
    plt.title('Gazebo Real-Time Factor Over Time')
    plt.grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(output_dir / 'rtf.png', bbox_inches='tight')
    plt.close(figure)

    figure = plt.figure()
    plt.plot(time_values, drone_z)
    plt.xlabel('time (s)')
    plt.ylabel('altitude (m)')
    plt.title('Drone Altitude Over Time')
    plt.grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(output_dir / 'altitude.png', bbox_inches='tight')
    plt.close(figure)

    duration_s = float(data['time_s'].iloc[-1] - data['time_s'].iloc[0])
    rtf_series = pd.to_numeric(data['rtf'], errors='coerce').dropna()
    summary_lines = [
        f'telemetry_csv: {csv_path}',
        f'samples: {len(data.index)}',
        f'duration_s: {duration_s:.2f}',
        f'car_message_rate_mean_hz: {data["car_message_rate_hz"].mean():.2f}',
        f'drone_altitude_min_m: {data["drone_z"].min():.2f}',
        f'drone_altitude_max_m: {data["drone_z"].max():.2f}',
    ]
    if 'drone_message_rate_hz' in data.columns:
        summary_lines.append(f'drone_message_rate_mean_hz: {data["drone_message_rate_hz"].mean():.2f}')
    if not rtf_series.empty:
        summary_lines.extend([
            f'rtf_min: {rtf_series.min():.2f}',
            f'rtf_mean: {rtf_series.mean():.2f}',
            f'rtf_max: {rtf_series.max():.2f}',
        ])
    if {'waypoint_x', 'waypoint_y'}.issubset(data.columns):
        horizontal_error = (
            (data['drone_x'] - data['waypoint_x']) ** 2
            + (data['drone_y'] - data['waypoint_y']) ** 2
        ).pow(0.5)
        finite_error = horizontal_error.replace([math.inf, -math.inf], pd.NA).dropna()
        if not finite_error.empty:
            summary_lines.append(f'waypoint_tracking_error_mean_m: {finite_error.mean():.2f}')
            summary_lines.append(f'waypoint_tracking_error_max_m: {finite_error.max():.2f}')

    (output_dir / 'summary.txt').write_text('\n'.join(summary_lines) + '\n', encoding='utf-8')

    print(f'plots written to {output_dir}')
    return 0


def main() -> int:
    if len(sys.argv) not in (2, 3):
        print('usage: tools/plot_run.py <telemetry_csv> [output_dir]', file=sys.stderr)
        return 1

    csv_path = Path(sys.argv[1])
    if not csv_path.exists():
        print(f'telemetry file not found: {csv_path}', file=sys.stderr)
        return 1

    output_dir = Path(sys.argv[2]) if len(sys.argv) == 3 else Path('plots')
    return plot_run(csv_path, output_dir)


if __name__ == '__main__':
    raise SystemExit(main())
