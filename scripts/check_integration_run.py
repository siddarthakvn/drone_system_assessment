#!/usr/bin/env python3
"""Validate a recorded integration-test run against assessment thresholds."""
import sys
from datetime import datetime
from pathlib import Path

import pandas as pd

TOOLS_DIR = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS_DIR))

from log_summary import EVENT_RE, summarize_log  # noqa: E402


def parse_iso_timestamp(value: str) -> datetime:
    if value.endswith('Z'):
        value = value[:-1] + '+00:00'
    return datetime.fromisoformat(value)


def check_telemetry(
    csv_path: Path,
    window_s: float = 30.0,
    shutdown_tail_s: float = 3.0,
    min_altitude_m: float = 1.0,
) -> None:
    data = pd.read_csv(csv_path)
    if data.empty:
        raise SystemExit('telemetry CSV is empty')

    airborne = data[data['drone_z'] > min_altitude_m]
    if airborne.empty:
        raise SystemExit(f'drone never exceeded {min_altitude_m:.2f} m during the run')

    end_time = float(airborne['time_s'].max())
    start_time = max(float(airborne['time_s'].min()), end_time - window_s)
    stop_time = max(start_time, end_time - shutdown_tail_s)
    window = airborne[(airborne['time_s'] >= start_time) & (airborne['time_s'] <= stop_time)]
    if window.empty:
        raise SystemExit(
            f'no airborne telemetry samples in final {window_s:.0f}s window '
            f'(excluding last {shutdown_tail_s:.0f}s shutdown tail)'
        )

    min_z = float(window['drone_z'].min())
    print(
        f'telemetry_ok: samples={len(window.index)} '
        f'window_s={window_s:.0f} min_drone_z={min_z:.2f}'
    )


def check_events(events_path: Path, window_s: float = 30.0, shutdown_tail_s: float = 3.0) -> None:
    if not events_path.exists():
        raise SystemExit(f'events log not found: {events_path}')

    timestamps = []
    errors_in_window = []
    for line in events_path.read_text(encoding='utf-8').splitlines():
        match = EVENT_RE.search(line)
        if not match:
            continue
        timestamp = parse_iso_timestamp(match.group('timestamp'))
        timestamps.append(timestamp)
        if match.group('severity') == 'ERROR':
            errors_in_window.append((timestamp, match.group('event_type')))

    if not timestamps:
        raise SystemExit('events.log contains no structured events')

    end_time = max(timestamps)
    start_time = end_time.timestamp() - window_s
    stop_time = end_time.timestamp() - shutdown_tail_s
    late_errors = [
        event_type
        for timestamp, event_type in errors_in_window
        if start_time <= timestamp.timestamp() <= stop_time
    ]
    if late_errors:
        raise SystemExit(
            f'found {len(late_errors)} ERROR events in final {window_s:.0f}s: '
            + ', '.join(sorted(set(late_errors)))
        )

    print(f'events_ok: no ERROR events in final {window_s:.0f}s')


def main() -> int:
    if len(sys.argv) != 2:
        print('usage: scripts/check_integration_run.py <run_directory>', file=sys.stderr)
        return 1

    run_dir = Path(sys.argv[1])
    telemetry = run_dir / 'telemetry.csv'
    events = run_dir / 'events.log'

    if not telemetry.exists():
        raise SystemExit(f'telemetry file not found: {telemetry}')

    check_telemetry(telemetry)
    check_events(events)

    print('integration checks passed')
    print('--- log summary ---')
    summarize_log(events if events.exists() else run_dir)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
