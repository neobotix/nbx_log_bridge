"""DDS integration test for transient-local retained log delivery."""

import time

from nbx_log_bridge.log_bridge import NbxLogBridge, output_qos, rosout_qos

from rcl_interfaces.msg import Log

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def spin_until(executor, predicate, timeout=5.0):
    """Spin until a condition becomes true or the timeout expires."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def test_late_transient_local_subscriber_receives_retained_record():
    """Deliver a retained error to a subscriber created after publication."""
    rclpy.init()
    bridge = NbxLogBridge()
    source = Node('nbx_log_bridge_test_source', enable_rosout=False)
    source_publisher = source.create_publisher(Log, '/rosout', rosout_qos())
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(source)
    early_subscriber = Node(
        'nbx_log_bridge_test_early_subscriber', enable_rosout=False)
    early_received = []
    early_subscriber.create_subscription(
        Log,
        '/nbx_lemma/logs',
        early_received.append,
        output_qos(500),
    )
    executor.add_node(early_subscriber)
    late_subscriber = None

    try:
        assert spin_until(
            executor,
            lambda: source_publisher.get_subscription_count() > 0,
        )

        record = Log()
        record.level = Log.ERROR
        record.name = 'startup_test'
        record.msg = 'emitted before Lemma connected'
        source_publisher.publish(record)
        assert spin_until(executor, lambda: len(early_received) == 1)

        executor.remove_node(early_subscriber)
        early_subscriber.destroy_node()
        early_subscriber = None

        received = []
        late_subscriber = Node(
            'nbx_log_bridge_test_late_subscriber', enable_rosout=False)
        late_subscriber.create_subscription(
            Log,
            '/nbx_lemma/logs',
            received.append,
            output_qos(500),
        )
        executor.add_node(late_subscriber)

        assert spin_until(executor, lambda: len(received) == 1)
        assert received[0].level == Log.ERROR
        assert received[0].name == 'startup_test'
        assert received[0].msg == 'emitted before Lemma connected'
    finally:
        if late_subscriber is not None:
            executor.remove_node(late_subscriber)
            late_subscriber.destroy_node()
        if early_subscriber is not None:
            executor.remove_node(early_subscriber)
            early_subscriber.destroy_node()
        executor.remove_node(source)
        executor.remove_node(bridge)
        source.destroy_node()
        bridge.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
