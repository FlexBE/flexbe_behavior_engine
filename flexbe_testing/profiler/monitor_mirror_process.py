#!/usr/bin/env python3

"""Run FlexBE mirror in its own process and print subscribed messages in arrival order."""

import argparse
import os
import sys
import threading
import time

from flexbe_core.core.topics import Topics

from flexbe_mirror.flexbe_mirror import FlexbeMirror

from flexbe_msgs.msg import BEStatus, StateMapMsg

import rclpy
from rclpy._rclpy_pybind11 import InvalidHandle
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


def _write_tty_status(message):
    """Write a progress message directly to the controlling terminal if available."""
    try:
        with open('/dev/tty', 'w', encoding='utf-8') as tty:
            print(message, file=tty)
    except OSError:
        pass


class MessagePrinter:
    """Print subscribed messages with a sequence number and relative timestamp."""

    def __init__(self):
        self._lock = threading.Lock()
        self._sequence = 0
        self._started_at = time.perf_counter()

    def _print(self, label, msg):
        with self._lock:
            self._sequence += 1
            elapsed = time.perf_counter() - self._started_at
            print(f'[{self._sequence:05d}] +{elapsed:9.6f}s {label}: {msg}', flush=True)

    def status_callback(self, msg):
        """Print mirror BEStatus messages."""
        self._print('mirror_status', msg)

    def state_map_callback(self, msg):
        """Print mirror state-map messages."""
        self._print('state_map', msg)


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=int, default=3, help='Accepted for compatibility; unused.')
    parser.add_argument('--warmup-runs', type=int, default=1, help='Accepted for compatibility; unused.')
    parser.add_argument('--startup-timeout', type=float, default=120.0,
                        help='Accepted for compatibility; unused.')
    parser.add_argument('--timeout', type=float, default=30.0, help='Accepted for compatibility; unused.')
    parser.add_argument('--profile-output', default='',
                        help='Accepted for compatibility; unused.')
    parser.add_argument('--profile-sort', default='cumulative',
                        help='Accepted for compatibility; unused.')
    parser.add_argument('--profile-top', type=int, default=25,
                        help='Accepted for compatibility; unused.')
    return parser.parse_args()


def main():
    """Run the standalone mirror process and print incoming subscribed messages."""
    _parse_args()
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

    rclpy.init()
    executor = SingleThreadedExecutor()
    helper_node = rclpy.create_node('mirror_monitor_process_driver')
    printer = MessagePrinter()
    mirror = FlexbeMirror()

    executor.add_node(helper_node)
    executor.add_node(mirror)

    status_qos = QoSProfile(depth=20, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    status_sub = helper_node.create_subscription(
        BEStatus,
        Topics._MIRROR_STATUS_TOPIC,
        printer.status_callback,
        status_qos,
    )
    state_map_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    state_map_sub = helper_node.create_subscription(
        StateMapMsg,
        Topics._STATE_MAP_OCS_TOPIC,
        printer.state_map_callback,
        state_map_qos,
    )

    _write_tty_status('Mirror monitor ready; streaming subscribed messages until interrupted.')

    try:
        while rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except KeyboardInterrupt:
                break
            except ExternalShutdownException:
                break
            except InvalidHandle:
                if not rclpy.ok():
                    break
                time.sleep(0.01)
    finally:
        try:
            mirror.shutdown_mirror()
        except (AttributeError, InvalidHandle, RuntimeError) as exc:
            _write_tty_status(f'Mirror shutdown cleanup error: {type(exc).__name__}: {exc}')
        helper_node.destroy_subscription(state_map_sub)
        helper_node.destroy_subscription(status_sub)
        mirror.destroy_node()
        helper_node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
