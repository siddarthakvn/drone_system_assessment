#!/usr/bin/env python3
import collections
import re
import sys
from pathlib import Path


EVENT_RE = re.compile(
    r'(?P<timestamp>\d{4}-\d{2}-\d{2}T[^ ]+) '
    r'(?P<severity>[A-Z]+) '
    r'(?P<component>\S+) '
    r'(?P<event_type>\S+) '
    r'(?P<description>.*)'
)


def iter_log_files(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    return sorted(child for child in path.rglob('*.log') if child.is_file())


def summarize_log(log_path: Path) -> int:
    log_files = iter_log_files(log_path)
    if not log_files:
        print(f'no log files found under: {log_path}', file=sys.stderr)
        return 1

    total_events = 0
    severity_counts = collections.Counter()
    component_counts = collections.Counter()
    component_severity_counts = collections.Counter()
    event_type_counts = collections.Counter()
    first_event_timestamp = None
    last_event_timestamp = None
    first_error_timestamp = None
    last_error_timestamp = None

    for file_path in log_files:
        for line in file_path.read_text(encoding='utf-8', errors='replace').splitlines():
            match = EVENT_RE.search(line)
            if not match:
                continue

            total_events += 1
            timestamp = match.group('timestamp')
            severity = match.group('severity')
            component = match.group('component')
            event_type = match.group('event_type')

            severity_counts[severity] += 1
            component_counts[component] += 1
            component_severity_counts[(component, severity)] += 1
            event_type_counts[(component, event_type)] += 1

            if first_event_timestamp is None:
                first_event_timestamp = timestamp
            last_event_timestamp = timestamp

            if severity == 'ERROR':
                if first_error_timestamp is None:
                    first_error_timestamp = timestamp
                last_error_timestamp = timestamp

    print(f'scanned_log_files: {len(log_files)}')
    print(f'total_events: {total_events}')
    print(f'total_warnings: {severity_counts.get("WARNING", 0)}')
    print(f'total_errors: {severity_counts.get("ERROR", 0)}')
    print(f'first_event_timestamp: {first_event_timestamp or "none"}')
    print(f'last_event_timestamp: {last_event_timestamp or "none"}')
    print(f'first_error_timestamp: {first_error_timestamp or "none"}')
    print(f'last_error_timestamp: {last_error_timestamp or "none"}')

    print('components:')
    for component, count in sorted(component_counts.items()):
        warnings = component_severity_counts.get((component, 'WARNING'), 0)
        errors = component_severity_counts.get((component, 'ERROR'), 0)
        print(f'  - {component}: events={count}, warnings={warnings}, errors={errors}')

    print('event_types:')
    for (component, event_type), count in sorted(event_type_counts.items()):
        print(f'  - {component}.{event_type}: {count}')

    return 0


def main() -> int:
    if len(sys.argv) != 2:
        print('usage: tools/log_summary.py <log_file_or_directory>', file=sys.stderr)
        return 1

    log_path = Path(sys.argv[1])
    if not log_path.exists():
        print(f'log path not found: {log_path}', file=sys.stderr)
        return 1

    return summarize_log(log_path)


if __name__ == '__main__':
    raise SystemExit(main())
