#!/usr/bin/env python3

"""Filter important ROS logs onto a retained, bounded DDS topic."""

from typing import Optional

from rcl_interfaces.msg import IntegerRange, Log, ParameterDescriptor

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


DEFAULT_HISTORY_DEPTH = 500
DEFAULT_MAX_MESSAGE_BYTES = 8192
MAX_HISTORY_DEPTH = 5_000
MAX_MESSAGE_BYTES = 65_536
MAX_RETENTION_BYTES = 64 * 1024 * 1024
ROSOUT_DEPTH = 1000
FORWARDED_LEVELS = frozenset((Log.WARN, Log.ERROR, Log.FATAL))


def validate_configuration(history_depth: int, max_message_bytes: int) -> None:
    """Validate bounds before allocating DDS history."""
    if isinstance(history_depth, bool) or not isinstance(history_depth, int):
        raise ValueError('history_depth must be an integer')
    if not 1 <= history_depth <= MAX_HISTORY_DEPTH:
        raise ValueError(
            f'history_depth must be between 1 and {MAX_HISTORY_DEPTH}')

    if (isinstance(max_message_bytes, bool) or
            not isinstance(max_message_bytes, int)):
        raise ValueError('max_message_bytes must be an integer')
    if not 1 <= max_message_bytes <= MAX_MESSAGE_BYTES:
        raise ValueError(
            f'max_message_bytes must be between 1 and {MAX_MESSAGE_BYTES}')

    retention_bytes = history_depth * max_message_bytes
    if retention_bytes > MAX_RETENTION_BYTES:
        raise ValueError(
            'history_depth * max_message_bytes must not exceed '
            f'{MAX_RETENTION_BYTES} bytes')


def truncate_utf8(text: str, max_bytes: int) -> str:
    """Return valid UTF-8 fitting within max_bytes."""
    encoded = text.encode('utf-8')
    if len(encoded) <= max_bytes:
        return text
    return encoded[:max_bytes].decode('utf-8', errors='ignore')


def filter_log(record: Log, max_message_bytes: int) -> Optional[Log]:
    """Copy a relevant record and safely bound its message text."""
    if record.level not in FORWARDED_LEVELS:
        return None

    forwarded = Log()
    forwarded.stamp = record.stamp
    forwarded.level = record.level
    forwarded.name = record.name
    forwarded.msg = truncate_utf8(record.msg, max_message_bytes)
    forwarded.file = record.file
    forwarded.function = record.function
    forwarded.line = record.line
    return forwarded


def rosout_qos() -> QoSProfile:
    """Match the standard ROS 2 Jazzy /rosout reliability and durability."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=ROSOUT_DEPTH,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def output_qos(history_depth: int) -> QoSProfile:
    """Create bounded transient-local history for live and late subscribers."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=history_depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _integer_descriptor(description: str, maximum: int) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=description,
        read_only=True,
        integer_range=[IntegerRange(from_value=1, to_value=maximum, step=1)],
    )


class NbxLogBridge(Node):
    """Forward warning-and-higher /rosout records to NBX Lemma."""

    def __init__(self) -> None:
        """Create the bounded publisher and begin listening to /rosout."""
        super().__init__('nbx_log_bridge')

        self.declare_parameter(
            'history_depth',
            DEFAULT_HISTORY_DEPTH,
            _integer_descriptor(
                'Maximum records retained in memory for late subscribers.',
                MAX_HISTORY_DEPTH,
            ),
        )
        self.declare_parameter(
            'max_message_bytes',
            DEFAULT_MAX_MESSAGE_BYTES,
            _integer_descriptor(
                'Maximum UTF-8 byte length of each forwarded Log.msg field.',
                MAX_MESSAGE_BYTES,
            ),
        )

        self._history_depth = self.get_parameter('history_depth').value
        self._max_message_bytes = self.get_parameter('max_message_bytes').value
        validate_configuration(self._history_depth, self._max_message_bytes)

        self._publisher = self.create_publisher(
            Log, '/nbx_lemma/logs', output_qos(self._history_depth))
        print(
            '[nbx_log_bridge] publisher created: '
            f'/nbx_lemma/logs (depth={self._history_depth}, '
            'reliable, transient_local)',
            flush=True,
        )

        print(
            '[nbx_log_bridge] creating /rosout subscription '
            '(depth=1000, reliable, transient_local)',
            flush=True,
        )
        self._subscription = self.create_subscription(
            Log, '/rosout', self._handle_log, rosout_qos())
        print('[nbx_log_bridge] /rosout subscription created', flush=True)

    def _handle_log(self, record: Log) -> None:
        print(
            '[nbx_log_bridge] received /rosout record: '
            f'level={record.level} name={record.name!r}',
            flush=True,
        )
        forwarded = filter_log(record, self._max_message_bytes)
        if forwarded is not None:
            print(
                '[nbx_log_bridge] forwarding record to /nbx_lemma/logs: '
                f'level={forwarded.level} name={forwarded.name!r}',
                flush=True,
            )
            self._publisher.publish(forwarded)
        else:
            print(
                '[nbx_log_bridge] ignored record below WARN severity',
                flush=True,
            )


def main(args=None) -> None:
    """Run the bridge until its ROS context shuts down."""
    rclpy.init(args=args)
    node = None
    try:
        node = NbxLogBridge()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
