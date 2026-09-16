"""Unit tests for filtering, copying, truncation, and configuration."""

from nbx_log_bridge.log_bridge import (
    DEFAULT_HISTORY_DEPTH,
    DEFAULT_MAX_MESSAGE_BYTES,
    MAX_HISTORY_DEPTH,
    MAX_MESSAGE_BYTES,
    MAX_RETENTION_BYTES,
    filter_log,
    output_qos,
    rosout_qos,
    validate_configuration,
)

import pytest

from rcl_interfaces.msg import Log

from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy


def make_log(level: int, message: str = 'diagnostic') -> Log:
    """Create a populated log record for filtering tests."""
    record = Log()
    record.stamp.sec = 123
    record.stamp.nanosec = 456
    record.level = level
    record.name = 'motor_controller'
    record.msg = message
    record.file = 'driver.py'
    record.function = 'poll'
    record.line = 78
    return record


@pytest.mark.parametrize('level', [Log.DEBUG, Log.INFO])
def test_low_severity_is_ignored(level):
    """Drop debug and informational records."""
    assert filter_log(make_log(level), 8192) is None


@pytest.mark.parametrize('level', [Log.WARN, Log.ERROR, Log.FATAL])
def test_important_severity_is_forwarded(level):
    """Forward warning, error, and fatal records."""
    result = filter_log(make_log(level), 8192)
    assert result is not None
    assert result.level == level


def test_original_metadata_is_preserved():
    """Copy every standard Log field without changing metadata."""
    original = make_log(Log.ERROR)

    result = filter_log(original, 8192)

    assert result is not original
    assert result.stamp == original.stamp
    assert result.level == original.level
    assert result.name == original.name
    assert result.msg == original.msg
    assert result.file == original.file
    assert result.function == original.function
    assert result.line == original.line


def test_oversized_text_is_truncated_at_a_utf8_boundary():
    """Never split a multi-byte UTF-8 code point."""
    original = make_log(Log.WARN, 'ab€€cd')

    result = filter_log(original, 6)

    assert result.msg == 'ab€'
    assert len(result.msg.encode('utf-8')) <= 6
    assert result.name == original.name


def test_output_qos_is_bounded_and_retained():
    """Configure reliable transient-local keep-last history."""
    qos = output_qos(37)

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 37
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_defaults_and_rosout_qos_are_explicit():
    """Keep documented defaults and request compatible /rosout QoS."""
    qos = rosout_qos()

    assert DEFAULT_HISTORY_DEPTH == 500
    assert DEFAULT_MAX_MESSAGE_BYTES == 8192
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 1000
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


@pytest.mark.parametrize(
    ('history_depth', 'max_message_bytes'),
    [
        (0, 8192),
        (-1, 8192),
        (MAX_HISTORY_DEPTH + 1, 8192),
        (500, 0),
        (500, -1),
        (500, MAX_MESSAGE_BYTES + 1),
        (MAX_HISTORY_DEPTH, MAX_MESSAGE_BYTES),
    ],
)
def test_invalid_parameter_bounds_are_rejected(
        history_depth, max_message_bytes):
    """Reject non-positive and unreasonable allocations."""
    with pytest.raises(ValueError):
        validate_configuration(history_depth, max_message_bytes)


def test_valid_parameter_bounds_are_accepted():
    """Accept individual limits when the combined budget remains safe."""
    validate_configuration(1, 1)
    validate_configuration(MAX_HISTORY_DEPTH, DEFAULT_MAX_MESSAGE_BYTES)
    validate_configuration(DEFAULT_HISTORY_DEPTH, MAX_MESSAGE_BYTES)


def test_combined_retention_budget_boundary_is_accepted():
    """Accept a configuration exactly at the combined memory budget."""
    validate_configuration(1024, MAX_RETENTION_BYTES // 1024)
