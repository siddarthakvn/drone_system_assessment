from datetime import datetime, timezone


def iso_timestamp() -> str:
    """Return an ISO-8601 UTC timestamp suitable for audit logs."""
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def format_event(severity: str, component: str, event_type: str, description: str) -> str:
    """Format failure and status events in a stable plain-text shape."""
    return f"{iso_timestamp()} {severity.upper()} {component} {event_type} {description}"


def log_event(node, severity: str, component: str, event_type: str, description: str) -> None:
    """Send a consistently formatted event line to the ROS logger.

    We log every event through `info()` and keep the actual severity in the
    message body. This avoids rclpy callsite-severity conflicts when the same
    helper line is used for mixed severities from timers and worker callbacks.
    """
    message = format_event(severity, component, event_type, description)
    node.get_logger().info(message)
