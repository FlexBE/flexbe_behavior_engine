#!/usr/bin/env python3

# Copyright 2024 Philipp Schillinger, Team ViGIR, Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Philipp Schillinger, Team ViGIR, Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Test for onboard behaviors."""
import multiprocessing
import os
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import weakref
import zlib

from flexbe_core.core import StateMachine, TransitionError
from flexbe_core.core.topics import Topics
from flexbe_core.proxy import ProxySubscriberCached

from flexbe_msgs.msg import BehaviorLog, BehaviorModification, BehaviorSelection, BehaviorSync, BEStatus

from flexbe_onboard import FlexbeOnboard

import rclpy
from rclpy._rclpy_pybind11 import InvalidHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


class TestOnboard(unittest.TestCase):
    """Test for onboard behaviors."""

    def setUp(self):
        """Set up the onboard test."""
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        num_threads = max(2, multiprocessing.cpu_count() - 1)
        self.executor = MultiThreadedExecutor(num_threads=num_threads, context=self.context)
        self.node = rclpy.create_node('TestOnboard', context=self.context)
        self.executor.add_node(self.node)

        ProxySubscriberCached.initialize(self.node)

        status_qos = QoSProfile(depth=20, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        log_qos = QoSProfile(depth=200)
        self.sub = ProxySubscriberCached(inst_id=id(self))
        self.sub.subscribe(Topics._ONBOARD_STATUS_TOPIC, BEStatus, qos=status_qos, inst_id=id(self))
        self.sub.subscribe(Topics._BEHAVIOR_LOGGING_TOPIC, BehaviorLog, qos=log_qos, inst_id=id(self))
        self.sub.enable_buffer(Topics._ONBOARD_STATUS_TOPIC)
        self.sub.remove_last_msg(Topics._ONBOARD_STATUS_TOPIC, clear_buffer=True)
        # make sure that behaviors can be imported
        self._test_data_folder = os.path.dirname(os.path.realpath(__file__))
        sys.path.insert(0, self._test_data_folder)

        # run onboard and add custom test behaviors to onboard lib
        if not rclpy.ok():
            rclpy.init()
        self.onboard = FlexbeOnboard()
        self.executor.add_node(self.onboard)

        self.lib = self.onboard._behavior_lib
        self.lib._add_behavior_manifests(self._test_data_folder)

    def tearDown(self):
        """Tear down the onboard test."""
        self.sub.unsubscribe_topic(Topics._ONBOARD_STATUS_TOPIC, inst_id=id(self))
        self.sub.unsubscribe_topic(Topics._BEHAVIOR_LOGGING_TOPIC, inst_id=id(self))
        self.node.destroy_node()
        self.onboard.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)
        if rclpy.ok():
            rclpy.shutdown()

    def assertStatus(self, expected, timeout, message=''):
        """Assert that the expected onboard status is received before the timeout."""
        self._safe_spin_once(timeout_sec=0.01)
        for i in range(int(timeout * 100)):

            # Spin both nodes as needed
            self._safe_spin_once(timeout_sec=0.01)
            time.sleep(0.02)
            msg = None
            if self.sub.has_buffered(Topics._ONBOARD_STATUS_TOPIC):
                msg = self.sub.get_from_buffer(Topics._ONBOARD_STATUS_TOPIC)
                if self.sub.has_msg(Topics._ONBOARD_STATUS_TOPIC) and \
                        self.sub.get_last_msg(Topics._ONBOARD_STATUS_TOPIC) is msg:
                    self.sub.remove_last_msg(Topics._ONBOARD_STATUS_TOPIC)
            elif self.sub.has_msg(Topics._ONBOARD_STATUS_TOPIC):
                msg = self.sub.get_last_msg(Topics._ONBOARD_STATUS_TOPIC)
                self.sub.remove_last_msg(Topics._ONBOARD_STATUS_TOPIC)
            if msg is None:
                continue
            self.node.get_logger().info(f'assertStatus: msg= {str(msg)} expected={expected} - {message}?')
            self.assertEqual(msg.code, expected, msg=message)
            return msg
        else:
            raise AssertionError('Did not receive a status as required.')

    def clear_extra_heartbeat_ready_messages(self):
        """Clear heartbeat ready messages."""
        while self.sub.has_buffered(Topics._ONBOARD_STATUS_TOPIC):
            msg = self.sub.get_from_buffer(Topics._ONBOARD_STATUS_TOPIC)
            if msg.code == BEStatus.READY:
                self.node.get_logger().info(f'clearing READY msg={str(msg)}')
                continue
            self._safe_spin_once(timeout_sec=0.01)
        while self.sub.has_msg(Topics._ONBOARD_STATUS_TOPIC):
            msg = self.sub.get_last_msg(Topics._ONBOARD_STATUS_TOPIC)
            if msg.code != BEStatus.READY:
                break
            self.node.get_logger().info(f'clearing cached READY msg={str(msg)}')
            self.sub.remove_last_msg(Topics._ONBOARD_STATUS_TOPIC)

    def clear_behavior_log_messages(self, timeout_sec=0.5):
        """Drain cached and buffered behavior logs so each phase starts with a clean slate."""
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            saw_message = False
            while self.sub.has_buffered(Topics._BEHAVIOR_LOGGING_TOPIC):
                self.sub.get_from_buffer(Topics._BEHAVIOR_LOGGING_TOPIC)
                saw_message = True
            if self.sub.has_msg(Topics._BEHAVIOR_LOGGING_TOPIC):
                self.sub.remove_last_msg(Topics._BEHAVIOR_LOGGING_TOPIC, clear_buffer=True)
                saw_message = True
            if not saw_message:
                break
            if not self._safe_spin_once(timeout_sec=0.01):
                break

    def settle_after_failure(self, timeout_sec=0.5):
        """Drain asynchronous status/log traffic after an expected failure path."""
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            saw_message = False
            while self.sub.has_buffered(Topics._ONBOARD_STATUS_TOPIC):
                self.sub.get_from_buffer(Topics._ONBOARD_STATUS_TOPIC)
                saw_message = True
            if self.sub.has_msg(Topics._ONBOARD_STATUS_TOPIC):
                self.sub.remove_last_msg(Topics._ONBOARD_STATUS_TOPIC)
                saw_message = True
            self.clear_behavior_log_messages(timeout_sec=0.05)
            if not self._safe_spin_once(timeout_sec=0.01):
                break
            if not saw_message and not self.onboard._starting:
                break

    def _get_deterministic_behavior_key(self):
        """Return a deterministic behavior key from the current library contents."""
        be_key = next((key for key, meta in self.lib._behavior_lib.items()
                       if meta.get('package') == 'flexbe_onboard_test_data'
                       and meta.get('name') == 'Log Behavior Test'), None)
        if be_key is None:
            # Invalid-key prepare paths can refresh the library from installed packages
            # and discard test-only manifests loaded in setUp.
            self.lib._add_behavior_manifests(self._test_data_folder)
            be_key = next((key for key, meta in self.lib._behavior_lib.items()
                           if meta.get('package') == 'flexbe_onboard_test_data'
                           and meta.get('name') == 'Log Behavior Test'), None)
        self.assertIsNotNone(be_key, "Required behavior 'Log Behavior Test' was not found in library")
        return be_key

    def test_onboard_behaviors(self):
        """Test onboard behaviors."""
        self._safe_spin_once(timeout_sec=1)

        behavior_pub = self.node.create_publisher(BehaviorSelection, Topics._START_BEHAVIOR_TOPIC, 1)
        # wait for publisher2
        end_time = time.time() + 3.0
        while time.time() < end_time:
            self._safe_spin_once(timeout_sec=0.1)

        # wait for the initial status message
        self._assert_eventual_status(BEStatus.READY, timeout_sec=12.0, message='BE is ready')

        # send simple behavior request without checksum
        be_key, _ = self.lib.find_behavior('Log Behavior Test')
        self.assertIsNotNone(be_key, "Required behavior 'Log Behavior Test' was not found in library")
        request = BehaviorSelection()
        request.behavior_key = be_key
        request.autonomy_level = 255

        self.clear_extra_heartbeat_ready_messages()

        self.node.get_logger().info('Publish request ...')
        behavior_pub.publish(request)
        self._safe_spin_once(timeout_sec=0.1)

        self.node.get_logger().info('Check for expected error ...')
        self.assertStatus(BEStatus.ERROR, 2, 'Error - checksum test')
        self.settle_after_failure(timeout_sec=1.0)

        # send valid simple behavior request
        with open(self.lib.get_sourcecode_filepath(be_key)) as f:
            request.behavior_id = zlib.adler32(f.read().encode()) & 0x7fffffff
        self.sub.enable_buffer(Topics._BEHAVIOR_LOGGING_TOPIC)
        self.clear_behavior_log_messages(timeout_sec=1.0)

        self.clear_extra_heartbeat_ready_messages()

        self.node.get_logger().info('Publish with checksum ...')
        behavior_pub.publish(request)
        self._safe_spin_once(timeout_sec=0.1)

        self.assertStatus(BEStatus.STARTED, 1, 'Started simple log behavior')
        self.assertStatus(BEStatus.FINISHED, 3, 'Finished simple log behavior')
        behavior_logs = self._collect_behavior_logs(timeout_sec=1.0)
        self._assert_eventual_behavior_log_contains('Test data',
                                                    timeout_sec=2.0,
                                                    initial_logs=behavior_logs)

        # send valid complex behavior request
        self.node.get_logger().info('Request to find (INVALID) complex behavior ...')
        be_key, _ = self.lib.find_behavior('Complex Behavior Test')
        request = BehaviorSelection()
        request.behavior_key = be_key
        request.autonomy_level = 255
        request.arg_keys = ['param']
        request.arg_values = ['value_2']
        request.input_keys = ['data']
        request.input_values = ['2']

        with open(self.lib.get_sourcecode_filepath(be_key)) as f:
            content = f.read()
        self.node.get_logger().info('Request behavior modification of (INVALID) complex behavior ...')
        modifications = [('INVALID', 'core'), ("raise ValueError('TODO: Remove!')", '')]
        for replace, by in modifications:
            index = content.index(replace)
            request.modifications.append(BehaviorModification(index_begin=index, index_end=index + len(replace), new_content=by))
        for replace, by in modifications:
            content = content.replace(replace, by)

        self.node.get_logger().info('Modified modified behavior ...')
        self.node.get_logger().info(content)
        self.node.get_logger().info(30 * '=' + '\n\n')
        request.behavior_id = zlib.adler32(content.encode()) & 0x7fffffff

        self.clear_extra_heartbeat_ready_messages()

        self.node.get_logger().info('Publish modified behavior ...')
        behavior_pub.publish(request)
        self._safe_spin_once(timeout_sec=0.1)
        self.assertStatus(BEStatus.STARTED, 1, 'Started modified')
        result = self.assertStatus(BEStatus.FINISHED, 3, 'Finished modified')
        self.assertEqual(result.args[0], 'finished')
        self._assert_eventual_behavior_log_contains('value_2', timeout_sec=2.0)

        self.clear_extra_heartbeat_ready_messages()

        # send the same behavior with different parameters
        self.node.get_logger().info('\n\nRepublish modified behavior ...')
        request.arg_keys = ['param', 'invalid']
        request.arg_values = ['value_1', 'should be ignored']
        request.input_keys = []
        request.input_values = []
        behavior_pub.publish(request)
        self._safe_spin_once(timeout_sec=0.1)
        self.assertStatus(BEStatus.STARTED, 1, 'Started modified parameters')
        result = self.assertStatus(BEStatus.FINISHED, 3, 'Finished modified parameters')
        self.assertEqual(result.args[0], 'failed')
        self.node.get_logger().info('\n\nExecute modified behavior ...')
        behavior_logs = self._collect_behavior_logs(timeout_sec=2.0)
        self.node.get_logger().info(f'{behavior_logs}')
        self._assert_eventual_behavior_log_contains('value_1',
                                                    timeout_sec=2.0,
                                                    initial_logs=behavior_logs)
        self.node.get_logger().info('Done onboard testing!')
        self._safe_spin_once(timeout_sec=0.1)

    def _safe_spin_once(self, timeout_sec=0.05):
        try:
            self.executor.spin_once(timeout_sec=timeout_sec)
            return True
        except InvalidHandle as exc:
            if 'cannot use Destroyable because destruction was requested' in str(exc):
                return False
            raise

    def _collect_behavior_logs(self, timeout_sec=1.0):
        logs = []
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            if not self._safe_spin_once(timeout_sec=0.05):
                break
            while self.sub.has_buffered(Topics._BEHAVIOR_LOGGING_TOPIC):
                logs.append(self.sub.get_from_buffer(Topics._BEHAVIOR_LOGGING_TOPIC).text)
        return logs

    def _assert_eventual_behavior_log_contains(self,
                                               expected_text,
                                               timeout_sec=2.0,
                                               initial_logs=None):
        """Assert that a behavior log is observed before timeout, even if teardown races occur."""
        logs = list(initial_logs) if initial_logs else []
        if expected_text in logs:
            return logs

        end_time = time.time() + timeout_sec
        saw_destroyable_race = False
        while time.time() < end_time:
            if not self._safe_spin_once(timeout_sec=0.05):
                saw_destroyable_race = True
            while self.sub.has_buffered(Topics._BEHAVIOR_LOGGING_TOPIC):
                logs.append(self.sub.get_from_buffer(Topics._BEHAVIOR_LOGGING_TOPIC).text)
                if expected_text in logs:
                    return logs
            if saw_destroyable_race:
                time.sleep(0.01)

        if saw_destroyable_race:
            raise AssertionError(f'{expected_text!r} not found before teardown race; logs={logs}')
        raise AssertionError(f'{expected_text!r} not found in {logs}')

    def _assert_eventual_error_status(self, timeout_sec=5.0, message=''):
        """Consume queued statuses until BEStatus.ERROR is seen or timeout occurs."""
        return self._assert_eventual_status(BEStatus.ERROR, timeout_sec=timeout_sec, message=message)

    def _assert_eventual_status(self, expected_code, timeout_sec=5.0, message=''):
        """Consume queued statuses until expected status code is seen or timeout occurs."""
        end_time = time.time() + timeout_sec
        last_msg = None
        while time.time() < end_time:
            if not self._safe_spin_once(timeout_sec=0.05):
                break
            msg = None
            if self.sub.has_buffered(Topics._ONBOARD_STATUS_TOPIC):
                msg = self.sub.get_from_buffer(Topics._ONBOARD_STATUS_TOPIC)
                if self.sub.has_msg(Topics._ONBOARD_STATUS_TOPIC) and \
                        self.sub.get_last_msg(Topics._ONBOARD_STATUS_TOPIC) is msg:
                    self.sub.remove_last_msg(Topics._ONBOARD_STATUS_TOPIC)
            elif self.sub.has_msg(Topics._ONBOARD_STATUS_TOPIC):
                msg = self.sub.get_last_msg(Topics._ONBOARD_STATUS_TOPIC)
                self.sub.remove_last_msg(Topics._ONBOARD_STATUS_TOPIC)
            if msg is None:
                continue
            last_msg = msg
            if msg.code == expected_code:
                return msg
        if last_msg is None:
            raise AssertionError(message or f'Did not receive BEStatus.{expected_code} before timeout')
        raise AssertionError(f'{message} (last status was code={last_msg.code})')

    def test_prepare_behavior_failure_reports_error(self):
        """Test direct prepare behavior failures emit ERROR status."""
        # Case 1: invalid key lookup
        request = BehaviorSelection()
        request.behavior_key = -999
        request.behavior_id = 424242
        request.autonomy_level = 255

        self.clear_extra_heartbeat_ready_messages()
        result = self.onboard._prepare_behavior(request)
        self.assertIsNone(result)
        self._assert_eventual_error_status(2.0, 'Prepare invalid key should publish ERROR')
        self.settle_after_failure(timeout_sec=1.0)

        # Case 2: checksum mismatch
        be_key = self._get_deterministic_behavior_key()
        request = BehaviorSelection()
        request.behavior_key = be_key
        request.behavior_id = 1  # Intentional mismatch with source checksum.
        request.autonomy_level = 255

        self.clear_extra_heartbeat_ready_messages()
        result = self.onboard._prepare_behavior(request)
        self.assertIsNone(result)
        self._assert_eventual_error_status(2.0, 'Prepare checksum mismatch should publish ERROR')
        self.settle_after_failure(timeout_sec=1.0)

        # Case 3: mismatched parameter arrays
        be_key = self._get_deterministic_behavior_key()
        request = BehaviorSelection()
        request.behavior_key = be_key
        request.autonomy_level = 255
        request.arg_keys = ['param']
        request.arg_values = []
        with open(self.lib.get_sourcecode_filepath(be_key)) as f:
            request.behavior_id = zlib.adler32(f.read().encode()) & 0x7fffffff

        self.clear_extra_heartbeat_ready_messages()
        result = self.onboard._prepare_behavior(request)
        self.assertIsNone(result)
        self._assert_eventual_error_status(2.0, 'Prepare mismatched parameter arrays should publish ERROR')
        self.settle_after_failure(timeout_sec=1.0)

        # Case 4: mismatched input arrays
        request = BehaviorSelection()
        request.behavior_key = be_key
        request.autonomy_level = 255
        request.input_keys = ['data']
        request.input_values = []
        with open(self.lib.get_sourcecode_filepath(be_key)) as f:
            request.behavior_id = zlib.adler32(f.read().encode()) & 0x7fffffff

        self.clear_extra_heartbeat_ready_messages()
        result = self.onboard._prepare_behavior(request)
        self.assertIsNone(result)
        self._assert_eventual_error_status(2.0, 'Prepare mismatched input arrays should publish ERROR')
        self.settle_after_failure(timeout_sec=1.0)

    def test_behavior_execution_transition_error_reports_error(self):
        """Test execution-phase TransitionError maps to ERROR status."""
        request = BehaviorSelection()
        request.behavior_key = 1
        request.behavior_id = 515151
        request.autonomy_level = 255

        class _FakeBehavior:

            def __init__(self, beh_id):
                self.name = 'fake_behavior'
                self.beh_id = beh_id
                self.requested_state_id = None
                self.state_map_items = ([], [])

            def confirm(self):
                return None

            def execute(self):
                raise TransitionError('forced transition error path')

        fake_behavior = _FakeBehavior(request.behavior_id)
        orig_prepare = self.onboard._prepare_behavior
        try:
            self.onboard._prepare_behavior = lambda msg: fake_behavior
            self.clear_extra_heartbeat_ready_messages()
            self.onboard._behavior_execution(request)
        finally:
            self.onboard._prepare_behavior = orig_prepare

        self._assert_eventual_status(BEStatus.ERROR, timeout_sec=2.0,
                                     message='TransitionError in execute should publish ERROR')


class TestOnboardCleanup(unittest.TestCase):
    """Unit tests for onboard temporary behavior cleanup."""

    def test_behavior_log_assertion_fails_on_teardown_race(self):
        """Missing expected logs must still fail when a destroyable teardown race occurs."""
        onboard_test = object.__new__(TestOnboard)

        class _FakeLogger:

            def warning(self, _msg):
                return None

            def info(self, _msg):
                return None

        class _FakeNode:

            @staticmethod
            def get_logger():
                return _FakeLogger()

        class _FakeSubscriber:

            @staticmethod
            def has_buffered(_topic):
                return False

            @staticmethod
            def get_from_buffer(_topic):
                raise AssertionError('No buffered log should be consumed')

        onboard_test.node = _FakeNode()
        onboard_test.sub = _FakeSubscriber()
        onboard_test._safe_spin_once = lambda timeout_sec=0.05: False

        with self.assertRaisesRegex(AssertionError, 'not found before teardown race'):
            onboard_test._assert_eventual_behavior_log_contains('missing', timeout_sec=0.01)

    def test_cleanup_behavior_removes_temp_python_sources_and_cache(self):
        """Temporary behavior cleanup should remove both source and cached bytecode."""
        onboard = object.__new__(FlexbeOnboard)
        with tempfile.TemporaryDirectory() as tmpdir:
            onboard._tmp_folder = tmpdir
            behavior_id = 12345
            source_path = os.path.join(tmpdir, f'tmp_{behavior_id}.py')
            pyc_path = os.path.join(tmpdir, f'tmp_{behavior_id}.pyc')
            pycache_dir = os.path.join(tmpdir, '__pycache__')
            cached_path = os.path.join(pycache_dir, f'tmp_{behavior_id}.cpython-312.pyc')

            os.makedirs(pycache_dir)
            for path in (source_path, pyc_path, cached_path):
                with open(path, 'w', encoding='utf-8') as handle:
                    handle.write('x')

            onboard._cleanup_behavior(behavior_id)

            self.assertFalse(os.path.exists(source_path))
            self.assertFalse(os.path.exists(pyc_path))
            self.assertFalse(os.path.exists(cached_path))

    def test_failed_switch_cleans_up_prepared_behavior(self):
        """Rejected switch attempts should clean up the newly prepared behavior artifacts."""
        onboard = object.__new__(FlexbeOnboard)
        cleaned_behavior_ids = []
        cleared_imports = []
        status_messages = []
        feedback_messages = []

        class _FakeLock:

            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, tb):
                return False

        class _FakePublisher:

            def publish(self, _topic, msg):
                feedback_messages.append(msg)

        class _FakeStatusPublisher:

            def publish(self, msg):
                status_messages.append(msg)

        class _ExistingBehavior:
            name = 'running'
            beh_id = 111

        class _PreparedBehavior:
            name = 'new'
            beh_id = 222

        onboard._enable_clear_imports = True
        onboard._clear_imports = lambda: cleared_imports.append(True)
        onboard._cleanup_behavior = lambda behavior_id: cleaned_behavior_ids.append(behavior_id)
        onboard._prepare_behavior = lambda msg: _PreparedBehavior()
        onboard._is_switchable = lambda be: False
        onboard._proxy_pub = _FakePublisher()
        onboard._status_pub = _FakeStatusPublisher()
        onboard._switch_lock = _FakeLock()
        onboard._run_lock = _FakeLock()
        onboard._running = True
        onboard._starting = True
        onboard._switching = False
        onboard.be = _ExistingBehavior()
        onboard._ready_counter = 0

        request = BehaviorSelection()
        request.behavior_key = 1
        request.behavior_id = 222

        with patch('flexbe_onboard.flexbe_onboard.rclpy.ok', return_value=True), \
                patch('flexbe_onboard.flexbe_onboard.Logger.logerr'), \
                patch('flexbe_onboard.flexbe_onboard.Logger.localinfo'), \
                patch('flexbe_onboard.flexbe_onboard.Logger.loginfo'):
            onboard._behavior_execution(request)

        self.assertEqual(cleaned_behavior_ids, [222])
        self.assertEqual(cleared_imports, [True])
        self.assertFalse(onboard._starting)
        self.assertFalse(onboard._switching)
        self.assertFalse(status_messages)
        self.assertEqual(feedback_messages[-1].args, ['not_switchable'])

    def test_prepare_behavior_uses_manifest_class_name(self):
        """Behavior preparation should instantiate the manifest-declared class, not the first sorted class."""
        onboard = object.__new__(FlexbeOnboard)
        with tempfile.TemporaryDirectory() as tmpdir:
            source_path = os.path.join(tmpdir, 'test_behavior.py')
            sys.path.insert(0, tmpdir)
            with open(source_path, 'w', encoding='utf-8') as handle:
                handle.write(
                    'class AHelper:\n'
                    '    def __init__(self, node):\n'
                    '        self.name = "helper"\n'
                    '        self.beh_id = 0\n'
                    'class ZBehavior:\n'
                    '    def __init__(self, node):\n'
                    '        self.name = "real"\n'
                    '        self.beh_id = 0\n'
                    '    def set_parameter(self, name, value):\n'
                    '        return False\n'
                    '    def set_up(self, beh_id, autonomy_level, debug):\n'
                    '        self.beh_id = beh_id\n'
                    '    def prepare_for_execution(self, input_data):\n'
                    '        self.input_data = input_data\n'
                )

            class _FakeLibrary:

                def get_behavior(self, _key):
                    return {'package': 'test_package', 'class': 'ZBehavior'}

                def get_sourcecode_filepath(self, _key, add_tmp=False):
                    return os.path.join(tmpdir, 'missing_tmp.py') if add_tmp else source_path

            class _FakeClock:

                class _Now:

                    @staticmethod
                    def to_msg():
                        return None

                @staticmethod
                def now():
                    return _FakeClock._Now()

            class _FakeLogger:

                def info(self, _msg):
                    return None

                def warning(self, _msg):
                    return None

                def error(self, _msg):
                    return None

            class _FakeStatusPublisher:

                def publish(self, _msg):
                    return None

            onboard._behavior_lib = _FakeLibrary()
            onboard._tmp_folder = tmpdir
            onboard._enable_clear_imports = False
            onboard._tracked_imports = []
            onboard._status_pub = _FakeStatusPublisher()
            onboard._clear_imports = lambda: None
            onboard.get_logger = lambda: _FakeLogger()
            onboard.get_clock = lambda: _FakeClock()

            request = BehaviorSelection()
            request.behavior_key = 1
            with open(source_path, encoding='utf-8') as handle:
                request.behavior_id = zlib.adler32(handle.read().encode()) & 0x7fffffff
            request.autonomy_level = 255

            try:
                with patch('flexbe_onboard.flexbe_onboard.Logger.loginfo'), \
                        patch('flexbe_onboard.flexbe_onboard.Logger.logerr'), \
                        patch('flexbe_onboard.flexbe_onboard.Logger.localinfo'):
                    behavior = onboard._prepare_behavior(request)
            finally:
                sys.path.remove(tmpdir)

            self.assertIsNotNone(behavior)
            self.assertEqual(type(behavior).__name__, 'ZBehavior')
            self.assertEqual(behavior.name, 'real')

    def test_prepare_behavior_rejects_manifest_class_imported_from_another_module(self):
        """Behavior preparation should reject manifest classes that are only re-exported into the temp module."""
        onboard = object.__new__(FlexbeOnboard)
        cleaned = []
        cleared = []
        published = []

        with tempfile.TemporaryDirectory() as tmpdir:
            source_path = os.path.join(tmpdir, 'test_behavior.py')
            helper_path = os.path.join(tmpdir, 'helper_module.py')
            sys.path.insert(0, tmpdir)
            with open(helper_path, 'w', encoding='utf-8') as handle:
                handle.write(
                    'class ImportedBehavior:\n'
                    '    def __init__(self, node):\n'
                    '        self.name = "imported"\n'
                    '        self.beh_id = 0\n'
                )
            with open(source_path, 'w', encoding='utf-8') as handle:
                handle.write('from helper_module import ImportedBehavior\n')

            class _FakeLibrary:

                def get_behavior(self, _key):
                    return {'package': 'test_package', 'class': 'ImportedBehavior'}

                def get_sourcecode_filepath(self, _key, add_tmp=False):
                    return os.path.join(tmpdir, 'missing_tmp.py') if add_tmp else source_path

            class _FakeClock:

                class _Now:

                    @staticmethod
                    def to_msg():
                        return None

                @staticmethod
                def now():
                    return _FakeClock._Now()

            onboard._behavior_lib = _FakeLibrary()
            onboard._tmp_folder = tmpdir
            onboard._enable_clear_imports = True
            onboard._tracked_imports = []
            onboard._clear_imports = lambda: cleared.append(True)
            onboard._cleanup_behavior = lambda behavior_id: cleaned.append(behavior_id)
            onboard._status_pub = type('_StatusPub', (), {'publish': staticmethod(lambda msg: published.append(msg))})()
            onboard.get_clock = lambda: _FakeClock()

            request = BehaviorSelection()
            request.behavior_key = 1
            with open(source_path, encoding='utf-8') as handle:
                request.behavior_id = zlib.adler32(handle.read().encode()) & 0x7fffffff
            request.autonomy_level = 255

            try:
                with patch('flexbe_onboard.flexbe_onboard.Logger.loginfo'), \
                        patch('flexbe_onboard.flexbe_onboard.Logger.logerr'), \
                        patch('flexbe_onboard.flexbe_onboard.Logger.localinfo'), \
                        patch('flexbe_onboard.flexbe_onboard.Logger.localwarn'):
                    behavior = onboard._prepare_behavior(request)
            finally:
                sys.path.remove(tmpdir)

        self.assertIsNone(behavior)
        self.assertEqual([True], cleared)
        self.assertEqual([request.behavior_id], cleaned)
        self.assertEqual(1, len(published))
        self.assertEqual(request.behavior_id, published[0].behavior_id)

    def test_prepare_behavior_reports_construction_failure_after_import(self):
        """Behavior construction failures after a valid import should clean up temp artifacts and publish ERROR."""
        onboard = object.__new__(FlexbeOnboard)
        cleaned = []
        cleared = []
        published = []

        with tempfile.TemporaryDirectory() as tmpdir:
            source_path = os.path.join(tmpdir, 'test_behavior.py')
            sys.path.insert(0, tmpdir)
            with open(source_path, 'w', encoding='utf-8') as handle:
                handle.write(
                    'class BuildBehavior:\n'
                    '    def __init__(self, node):\n'
                    '        self.name = "builder"\n'
                    '        self.beh_id = 0\n'
                    '    def set_parameter(self, name, value):\n'
                    '        return False\n'
                    '    def set_up(self, beh_id, autonomy_level, debug):\n'
                    '        self.beh_id = beh_id\n'
                    '    def prepare_for_execution(self, input_data):\n'
                    '        raise RuntimeError("construction failed")\n'
                )

            class _FakeLibrary:

                def get_behavior(self, _key):
                    return {'package': 'test_package', 'class': 'BuildBehavior'}

                def get_sourcecode_filepath(self, _key, add_tmp=False):
                    return os.path.join(tmpdir, 'missing_tmp.py') if add_tmp else source_path

            class _FakeClock:

                class _Now:

                    @staticmethod
                    def to_msg():
                        return None

                @staticmethod
                def now():
                    return _FakeClock._Now()

            onboard._behavior_lib = _FakeLibrary()
            onboard._tmp_folder = tmpdir
            onboard._enable_clear_imports = True
            onboard._tracked_imports = []
            onboard._clear_imports = lambda: cleared.append(True)
            onboard._cleanup_behavior = lambda behavior_id: cleaned.append(behavior_id)
            onboard._status_pub = type('_StatusPub', (), {'publish': staticmethod(lambda msg: published.append(msg))})()
            onboard.get_clock = lambda: _FakeClock()

            request = BehaviorSelection()
            request.behavior_key = 1
            with open(source_path, encoding='utf-8') as handle:
                request.behavior_id = zlib.adler32(handle.read().encode()) & 0x7fffffff
            request.autonomy_level = 255

            try:
                with patch('flexbe_onboard.flexbe_onboard.Logger.loginfo'), \
                        patch('flexbe_onboard.flexbe_onboard.Logger.logerr'), \
                        patch('flexbe_onboard.flexbe_onboard.Logger.localinfo'), \
                        patch('flexbe_onboard.flexbe_onboard.Logger.localwarn'):
                    behavior = onboard._prepare_behavior(request)
            finally:
                sys.path.remove(tmpdir)

        self.assertIsNone(behavior)
        self.assertEqual([True], cleared)
        self.assertEqual([request.behavior_id], cleaned)
        self.assertEqual(1, len(published))
        self.assertEqual(request.behavior_id, published[0].behavior_id)

    def test_verify_no_active_behaviors_clears_switching_flag(self):
        """Shutdown verification should clear stale switching state once no behavior is active."""
        onboard = object.__new__(FlexbeOnboard)
        onboard._run_lock = threading.Lock()
        onboard.be = None
        onboard._switching = True

        result = onboard.verify_no_active_behaviors(timeout=0.01)

        self.assertTrue(result)
        self.assertFalse(onboard._switching)

    def test_verify_no_active_behaviors_releases_lock_on_assertion(self):
        """Shutdown verification should release the run lock even if consistency assertion fails."""
        onboard = object.__new__(FlexbeOnboard)
        onboard._run_lock = threading.Lock()
        onboard.be = object()
        onboard._switching = True

        with self.assertRaises(AssertionError):
            onboard.verify_no_active_behaviors(timeout=0.01)

        self.assertFalse(onboard._run_lock.locked())

    def test_verify_no_active_behaviors_returns_false_when_lock_cannot_be_acquired(self):
        """Shutdown verification should return False when another thread still owns the run lock."""
        onboard = object.__new__(FlexbeOnboard)
        onboard.be = None
        onboard._switching = True
        onboard._run_lock = type('_BusyLock', (), {'acquire': staticmethod(lambda timeout=0.0: False)})()

        self.assertFalse(onboard.verify_no_active_behaviors(timeout=0.01))
        self.assertTrue(onboard._switching)

    def test_userdata_callback_uses_snapshotted_state_machine(self):
        """Userdata service should finish from a local snapshot even if self.be changes during the callback."""
        onboard = object.__new__(FlexbeOnboard)

        class _FakeLogger:

            def info(self, _msg):
                return None

        class _FakeUserdata:

            def __init__(self):
                self._data = {'foo': 42}

        class _FakeStateMachine:

            def __init__(self):
                self._name = 'fake_sm'
                self._userdata = _FakeUserdata()
                self._states = []

        class _FakeBehavior:

            def __init__(self):
                self._state_machine = _FakeStateMachine()

        class _Request:

            userdata_key = ''

        class _Response:

            def __init__(self):
                self.success = False
                self.message = ''
                self.userdata = []

        onboard.be = _FakeBehavior()
        onboard.get_logger = lambda: _FakeLogger()

        def _drop_behavior(state_machine, userdata, userdata_key, path):
            onboard.be = None
            return userdata

        onboard._get_userdata_from_whole_sm = _drop_behavior

        with patch('flexbe_onboard.flexbe_onboard.Logger.localinfo'):
            response = onboard._userdata_callback(_Request(), _Response())

        self.assertTrue(response.success)
        self.assertEqual(response.message, "Found 1 occurrences of '' from be='fake_sm'")
        self.assertEqual(len(response.userdata), 1)
        self.assertEqual(response.userdata[0].key, 'foo')

    def test_userdata_callback_reports_missing_state_machine_and_empty_key_matches(self):
        """Userdata service should fail cleanly when idle or when the requested key is absent."""
        onboard = object.__new__(FlexbeOnboard)
        onboard.be = None

        class _Request:

            def __init__(self, userdata_key):
                self.userdata_key = userdata_key

        class _Response:

            def __init__(self):
                self.success = False
                self.message = ''
                self.userdata = []

        idle_response = onboard._userdata_callback(_Request('foo'), _Response())
        self.assertFalse(idle_response.success)
        self.assertEqual('no state_machine running', idle_response.message)

        onboard.be = type(
            '_Behavior',
            (),
            {
                '_state_machine': type(
                    '_StateMachine',
                    (),
                    {
                        '_name': 'fake_sm',
                        '_userdata': type('_Userdata', (), {'_data': {'foo': 42}})(),
                        '_states': [],
                    },
                )(),
            },
        )()
        onboard._get_userdata_from_whole_sm = lambda state_machine, userdata, userdata_key, path: userdata

        with patch('flexbe_onboard.flexbe_onboard.Logger.localinfo'):
            filtered_response = onboard._userdata_callback(_Request('missing'), _Response())
        self.assertFalse(filtered_response.success)
        self.assertEqual("Found 0 occurrences of 'missing' from be='fake_sm'", filtered_response.message)
        self.assertEqual([], filtered_response.userdata)

    def test_report_prepare_failure_clears_imports_and_behavior_artifacts_when_requested(self):
        """Preparation failures should publish status and honor cleanup flags."""
        onboard = object.__new__(FlexbeOnboard)
        cleared = []
        cleaned = []
        published = []
        onboard._enable_clear_imports = True
        onboard._clear_imports = lambda: cleared.append(True)
        onboard._cleanup_behavior = lambda behavior_id: cleaned.append(behavior_id)
        onboard._status_pub = type('_StatusPub', (), {'publish': staticmethod(lambda msg: published.append(msg))})()
        onboard.get_clock = lambda: type(
            '_Clock',
            (),
            {'now': staticmethod(lambda: type('_Now', (), {'to_msg': staticmethod(lambda: None)})())},
        )()

        request = BehaviorSelection()
        request.behavior_id = 55

        with patch('flexbe_onboard.flexbe_onboard.Logger.logerr'):
            result = onboard._report_prepare_failure(
                request,
                RuntimeError('boom'),
                'failed preparing',
                clear_imports=True,
                cleanup_behavior=True,
            )

        self.assertIsNone(result)
        self.assertEqual([True], cleared)
        self.assertEqual([55], cleaned)
        self.assertEqual(1, len(published))
        self.assertEqual(55, published[0].behavior_id)

    def test_report_execution_failure_publishes_status_and_returns_fallback_result(self):
        """Execution failure reporting should publish the mapped status and preserve explicit results."""
        onboard = object.__new__(FlexbeOnboard)
        published = []
        onboard.be = type('_Behavior', (), {'beh_id': 77, 'name': 'demo'})()
        onboard._status_pub = type('_StatusPub', (), {'publish': staticmethod(lambda msg: published.append(msg))})()
        onboard.get_clock = lambda: type(
            '_Clock',
            (),
            {'now': staticmethod(lambda: type('_Now', (), {'to_msg': staticmethod(lambda: None)})())},
        )()

        with patch('flexbe_onboard.flexbe_onboard.Logger.logerr'), \
                patch('flexbe_onboard.flexbe_onboard.Logger.localinfo'), \
                patch('flexbe_onboard.flexbe_onboard.map_exception_to_bestatus', return_value=BEStatus.WARNING):
            fallback = onboard._report_execution_failure(RuntimeError('boom'), None)
            explicit = onboard._report_execution_failure(RuntimeError('boom'), 'kept')

        self.assertEqual('exception', fallback)
        self.assertEqual('kept', explicit)
        self.assertEqual(2, len(published))
        self.assertTrue(all(msg.behavior_id == 77 for msg in published))
        self.assertTrue(all(msg.code == BEStatus.WARNING for msg in published))

    def test_behavior_execution_aborts_when_rclpy_is_not_ok(self):
        """Behavior execution should stop immediately when ROS shutdown is already in progress."""
        onboard = object.__new__(FlexbeOnboard)
        cleaned = []
        prepared = []
        onboard._cleanup_tempdir = lambda: cleaned.append(True)
        onboard._prepare_behavior = lambda msg: prepared.append(msg)
        onboard._running = False
        onboard._starting = True
        onboard._switching = True

        request = BehaviorSelection()
        request.behavior_id = 123

        with patch('flexbe_onboard.flexbe_onboard.rclpy.ok', return_value=False):
            onboard._behavior_execution(request)

        self.assertEqual(cleaned, [True])
        self.assertEqual(prepared, [])
        self.assertFalse(onboard._starting)
        self.assertFalse(onboard._switching)

    def test_behavior_execution_prepare_failure_when_idle_rearms_ready_retry(self):
        """Preparation failure while idle should re-arm READY publication instead of sending switch feedback."""
        onboard = object.__new__(FlexbeOnboard)
        feedback_messages = []
        onboard._prepare_behavior = lambda _msg: None
        onboard._proxy_pub = type(
            '_ProxyPub',
            (),
            {'publish': staticmethod(lambda _topic, msg: feedback_messages.append(msg))},
        )()
        onboard._running = False
        onboard._starting = True
        onboard._switching = False
        onboard._ready_counter = 0

        request = BehaviorSelection()
        request.behavior_key = 1
        request.behavior_id = 222

        with patch('flexbe_onboard.flexbe_onboard.rclpy.ok', return_value=True), \
                patch('flexbe_onboard.flexbe_onboard.Logger.logerr'), \
                patch('flexbe_onboard.flexbe_onboard.Logger.localinfo'):
            onboard._behavior_execution(request)

        self.assertEqual([], feedback_messages)
        self.assertEqual(8, onboard._ready_counter)
        self.assertFalse(onboard._starting)
        self.assertFalse(onboard._switching)

    def test_behavior_execution_switch_prepare_failure_restores_running_status(self):
        """Switch preparation failures should keep the old behavior running and restore RUNNING status."""
        onboard = object.__new__(FlexbeOnboard)
        feedback_messages = []
        status_messages = []
        cleared_imports = []
        cleaned_behavior_ids = []

        class _FakeLock:

            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, tb):
                return False

        class _LeafState:
            name = 'leaf'

        active_container = object.__new__(StateMachine)
        active_container._name = 'container'
        active_leaf = _LeafState()

        class _CurrentBehavior:
            name = 'running'
            beh_id = 111

            @staticmethod
            def get_current_states():
                return [active_container, active_leaf]

            @staticmethod
            def preempt():
                raise AssertionError('switch prepare failure should not preempt the running behavior')

        class _PreparedBehavior:
            name = 'running'
            beh_id = 222

            @staticmethod
            def prepare_for_switch(_active_state):
                raise RuntimeError('cannot prepare switch')

        class _FakeClock:

            class _Now:

                @staticmethod
                def to_msg():
                    return None

            @staticmethod
            def now():
                return _FakeClock._Now()

        onboard._prepare_behavior = lambda _msg: _PreparedBehavior()
        onboard._is_switchable = lambda _be: True
        onboard._proxy_pub = type(
            '_ProxyPub',
            (),
            {'publish': staticmethod(lambda _topic, msg: feedback_messages.append(msg))},
        )()
        onboard._status_pub = type(
            '_StatusPub',
            (),
            {'publish': staticmethod(lambda msg: status_messages.append(msg))},
        )()
        onboard._switch_lock = _FakeLock()
        onboard._run_lock = _FakeLock()
        onboard._run_state_event = type(
            '_Event',
            (),
            {
                'clear': lambda self: None,
                'set': lambda self: None,
                'wait': lambda self, timeout=None: True,
            },
        )()
        onboard._enable_clear_imports = True
        onboard._clear_imports = lambda: cleared_imports.append(True)
        onboard._cleanup_behavior = lambda behavior_id: cleaned_behavior_ids.append(behavior_id)
        onboard._running = True
        onboard._starting = True
        onboard._switching = False
        onboard._ready_counter = 0
        onboard.be = _CurrentBehavior()
        onboard.get_clock = lambda: _FakeClock()

        request = BehaviorSelection()
        request.behavior_key = 1
        request.behavior_id = 222

        with patch('flexbe_onboard.flexbe_onboard.rclpy.ok', return_value=True), \
                patch('flexbe_onboard.flexbe_onboard.Logger.logerr'), \
                patch('flexbe_onboard.flexbe_onboard.Logger.loginfo'), \
                patch('flexbe_onboard.flexbe_onboard.Logger.localinfo'):
            onboard._behavior_execution(request)

        self.assertEqual(['received', 'start', 'failed'], [msg.args[0] for msg in feedback_messages])
        self.assertEqual([True], cleared_imports)
        self.assertEqual([222], cleaned_behavior_ids)
        self.assertEqual([BEStatus.SWITCHING, BEStatus.RUNNING], [msg.code for msg in status_messages])
        self.assertTrue(onboard._running)
        self.assertIsInstance(onboard.be, _CurrentBehavior)
        self.assertFalse(onboard._switching)
        self.assertFalse(onboard._starting)

    def test_heartbeat_worker_publishes_without_subscribers(self):
        """Heartbeat worker should publish heartbeat even when nobody is subscribed."""
        onboard = object.__new__(FlexbeOnboard)
        heartbeat_messages = []
        status_messages = []

        class _FakeHeartbeatPublisher:

            @staticmethod
            def get_subscription_count():
                return 0

            @staticmethod
            def publish(msg):
                heartbeat_messages.append(msg)

        class _FakeStatusPublisher:

            @staticmethod
            def get_subscription_count():
                return 1

            @staticmethod
            def publish(msg):
                status_messages.append(msg)

        class _FakeClock:

            class _Now:

                @staticmethod
                def to_msg():
                    return None

            @staticmethod
            def now():
                return _FakeClock._Now()

        onboard.be = None
        onboard._heartbeat_pub = _FakeHeartbeatPublisher()
        onboard._status_pub = _FakeStatusPublisher()
        onboard._idle_heartbeat = BehaviorSync()
        onboard._ready_status = BEStatus(code=BEStatus.READY)
        onboard._running = False
        onboard._switching = False
        onboard._trigger_ready = True
        onboard._ready_counter = 0
        onboard.get_clock = lambda: _FakeClock()

        with patch('flexbe_onboard.flexbe_onboard.Logger.check_local_enabled', lambda: None):
            onboard._heartbeat_worker()

        self.assertEqual(len(heartbeat_messages), 1)
        self.assertEqual(len(status_messages), 1)
        self.assertEqual(status_messages[0].code, BEStatus.READY)
        self.assertFalse(onboard._trigger_ready)
        self.assertEqual(onboard._ready_counter, 0)

    def test_startup_probe_callback_cancels_timer_and_logs_once(self):
        """Startup probe should cancel itself after proving timer callbacks are serviced."""
        onboard = object.__new__(FlexbeOnboard)
        canceled = []
        onboard._startup_probe_timer = type(
            '_Timer',
            (),
            {'cancel': staticmethod(lambda: canceled.append(True))},
        )()

        with patch('flexbe_onboard.flexbe_onboard.Logger.localinfo') as localinfo:
            onboard._startup_probe_callback()

        self.assertEqual(canceled, [True])
        localinfo.assert_called_once_with(
            'Onboard behavior engine active; publishers are initialized '
            'and executor is servicing timer callbacks.'
        )

    def test_publish_ready_status_reuses_cached_message(self):
        """READY publication should reuse the cached message shell and only refresh its stamp."""
        onboard = object.__new__(FlexbeOnboard)
        published = []

        class _FakeStatusPublisher:

            @staticmethod
            def get_subscription_count():
                return 1

            @staticmethod
            def publish(msg):
                published.append(msg)

        class _FakeClock:

            class _Now:

                @staticmethod
                def to_msg():
                    return object()

            @staticmethod
            def now():
                return _FakeClock._Now()

        onboard._status_pub = _FakeStatusPublisher()
        onboard._ready_status = BEStatus(code=BEStatus.READY)
        onboard.get_clock = lambda: _FakeClock()

        onboard._publish_ready_status()
        onboard._publish_ready_status()

        self.assertEqual(len(published), 2)
        self.assertIs(published[0], published[1])

    def test_publish_stopped_status_ignores_destroyed_status_publisher(self):
        """STOPPED publication should no-op if the status publisher is already being destroyed."""
        onboard = object.__new__(FlexbeOnboard)
        published = []
        onboard._status_pub = type(
            '_StatusPub',
            (),
            {
                'publish': staticmethod(lambda msg: (_ for _ in ()).throw(InvalidHandle('destruction was requested'))),
            },
        )()
        onboard.get_clock = lambda: type(
            '_Clock',
            (),
            {'now': staticmethod(lambda: type('_Now', (), {'to_msg': staticmethod(lambda: object())})())},
        )()

        onboard._publish_stopped_status(41)

        self.assertEqual(published, [])

    def test_behavior_switch_waits_on_run_state_event(self):
        """Switch preparation should wait on the shared run-state event instead of 1 ms polling."""
        onboard = object.__new__(FlexbeOnboard)
        status_messages = []
        feedback_messages = []
        state_map_messages = []
        wait_timeouts = []
        active_ready = {'value': False}
        prepared_from = []

        class _FakeLock:

            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, tb):
                return False

        class _FakeEvent:

            def clear(self):
                return None

            def set(self):  # noqa: A003
                return None

            def wait(self, timeout=None):
                wait_timeouts.append(timeout)
                active_ready['value'] = True
                return True

        class _FakePublisher:

            @staticmethod
            def number_of_subscribers(_topic):
                return 0

            def publish(self, *args):
                feedback_messages.append(args)

        class _FakeStatusPublisher:

            def publish(self, msg):
                status_messages.append(msg)

        class _FakeStateMapPublisher:

            def publish(self, msg):
                state_map_messages.append(msg)

        class _FakeClock:

            class _Now:

                @staticmethod
                def to_msg():
                    return None

            @staticmethod
            def now():
                return _FakeClock._Now()

        class _OldActiveState:
            name = 'active'

        active_container = object.__new__(StateMachine)
        active_container._name = 'container'

        class _OldBehavior:
            name = 'running'
            beh_id = 111

            @staticmethod
            def get_current_states():
                return [active_container, _OldActiveState()] if active_ready['value'] else None

            @staticmethod
            def preempt():
                onboard._running = False
                onboard.be = None
                onboard._run_state_event.set()

        class _PreparedBehavior:

            def __init__(self):
                self.name = 'new'
                self.beh_id = 222
                self.requested_state_id = None
                self.state_map_items = ([], [])

            def prepare_for_switch(self, active_state):
                prepared_from.append(active_state.name)

            def confirm(self):
                return None

            def execute(self):
                return 'done'

        onboard._prepare_behavior = lambda _msg: _PreparedBehavior()
        onboard._is_switchable = lambda _be: True
        onboard._proxy_pub = _FakePublisher()
        onboard._status_pub = _FakeStatusPublisher()
        onboard._state_map_pub = _FakeStateMapPublisher()
        onboard._switch_lock = _FakeLock()
        onboard._run_lock = _FakeLock()
        onboard._run_state_event = _FakeEvent()
        onboard._cleanup_behavior = lambda _behavior_id: None
        onboard._enable_clear_imports = False
        onboard._ready_counter = 0
        onboard._running = True
        onboard._starting = True
        onboard._switching = False
        onboard.be = _OldBehavior()
        onboard.get_clock = lambda: _FakeClock()

        request = BehaviorSelection()
        request.behavior_key = 1
        request.behavior_id = 222

        with patch('flexbe_onboard.flexbe_onboard.rclpy.ok', return_value=True), \
                patch('flexbe_onboard.flexbe_onboard.Logger.localinfo'), \
                patch('flexbe_onboard.flexbe_onboard.Logger.loginfo'), \
                patch('flexbe_onboard.flexbe_onboard.Logger.logwarn'), \
                patch('flexbe_onboard.flexbe_onboard.Logger.logerr'):
            onboard._behavior_execution(request)

        self.assertEqual(wait_timeouts, [0.05])
        self.assertEqual(prepared_from, ['active'])
        self.assertTrue(status_messages)
        self.assertTrue(state_map_messages)

    def test_behavior_switch_rejects_multiple_active_leaf_states(self):
        """Ambiguous multi-leaf switches should fail cleanly instead of guessing a container anchor."""
        onboard = object.__new__(FlexbeOnboard)
        status_messages = []
        feedback_messages = []
        prepared_from = []
        cleaned_behavior_ids = []

        class _FakeLock:

            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, tb):
                return False

        class _FakePublisher:

            @staticmethod
            def number_of_subscribers(_topic):
                return 0

            def publish(self, *args):
                feedback_messages.append(args)

        class _FakeStatusPublisher:

            def publish(self, msg):
                status_messages.append(msg)

        class _FakeStateMapPublisher:

            def publish(self, msg):
                return None

        class _FakeClock:

            class _Now:

                @staticmethod
                def to_msg():
                    return None

            @staticmethod
            def now():
                return _FakeClock._Now()

        class _LeafState:

            def __init__(self, name):
                self.name = name

        active_container = object.__new__(StateMachine)
        active_container._name = 'container'
        first_leaf = _LeafState('first')
        second_leaf = _LeafState('second')

        class _OldBehavior:
            name = 'running'
            beh_id = 111

            @staticmethod
            def get_current_states():
                return [active_container, first_leaf, second_leaf]

            @staticmethod
            def preempt():
                raise AssertionError('Old behavior should not be preempted for ambiguous multi-leaf switch')

        class _PreparedBehavior:

            def __init__(self):
                self.name = 'new'
                self.beh_id = 222
                self.requested_state_id = None
                self.state_map_items = ([], [])

            def prepare_for_switch(self, active_state):
                prepared_from.append(active_state.name)

        old_behavior = _OldBehavior()
        onboard._prepare_behavior = lambda _msg: _PreparedBehavior()
        onboard._is_switchable = lambda _be: True
        onboard._proxy_pub = _FakePublisher()
        onboard._status_pub = _FakeStatusPublisher()
        onboard._state_map_pub = _FakeStateMapPublisher()
        onboard._switch_lock = _FakeLock()
        onboard._run_lock = _FakeLock()
        onboard._run_state_event = type(
            '_Event',
            (),
            {
                'clear': lambda self: None,
                'set': lambda self: None,
                'wait': lambda self, timeout=None: True,
            },
        )()
        onboard._cleanup_behavior = lambda behavior_id: cleaned_behavior_ids.append(behavior_id)
        onboard._enable_clear_imports = False
        onboard._ready_counter = 0
        onboard._running = True
        onboard._starting = True
        onboard._switching = False
        onboard.be = old_behavior
        onboard.get_clock = lambda: _FakeClock()

        request = BehaviorSelection()
        request.behavior_key = 1
        request.behavior_id = 222

        with patch('flexbe_onboard.flexbe_onboard.rclpy.ok', return_value=True), \
                patch('flexbe_onboard.flexbe_onboard.Logger.localinfo'), \
                patch('flexbe_onboard.flexbe_onboard.Logger.loginfo'), \
                patch('flexbe_onboard.flexbe_onboard.Logger.logwarn'), \
                patch('flexbe_onboard.flexbe_onboard.Logger.logerr'):
            onboard._behavior_execution(request)

        self.assertEqual(prepared_from, [])
        self.assertEqual(cleaned_behavior_ids, [222])
        self.assertTrue(any(args[1].args == ['not_switchable'] for args in feedback_messages))
        self.assertEqual(BEStatus.RUNNING, status_messages[-1].code)
        self.assertTrue(onboard._running)
        self.assertIs(onboard.be, old_behavior)
        self.assertFalse(onboard._switching)
        self.assertFalse(onboard._starting)


class TestOnboardHelperCoverage(unittest.TestCase):
    """Target helper-style onboard branches without transport-heavy integration."""

    @staticmethod
    def _noop(*args, **kwargs):
        return None

    def test_version_parsing_and_warning_for_outdated_ui(self):
        """Version helpers should parse dotted versions and warn only when the UI is too old."""
        self.assertEqual(FlexbeOnboard._parse_version('2.3.4'), 20304)

        onboard = object.__new__(FlexbeOnboard)
        with patch('flexbe_onboard.flexbe_onboard.Logger.logwarn') as logwarn:
            onboard._version_callback(type('_Msg', (), {'data': '0.0.1'})())
            onboard._version_callback(type('_Msg', (), {'data': '999.0.0'})())

        logwarn.assert_called_once()

    def test_behavior_callback_starts_worker_thread_and_throttles_duplicates(self):
        """Behavior requests should start one worker thread and reject duplicates while starting."""
        launched = []

        class _Thread:

            def __init__(self, target, args):
                self.target = target
                self.args = args
                self.daemon = False

            def start(self):
                launched.append((self.target, self.args, self.daemon))

        onboard = object.__new__(FlexbeOnboard)
        onboard._start_lock = threading.Lock()
        onboard._starting = False
        onboard._trigger_ready = True
        onboard._ready_counter = 9
        onboard._behavior_execution = lambda msg: None

        msg = BehaviorSelection()
        msg.behavior_key = 7
        msg.behavior_id = 42

        with patch('flexbe_onboard.flexbe_onboard.threading.Thread', _Thread):
            onboard._behavior_callback(msg)

        self.assertTrue(onboard._starting)
        self.assertFalse(onboard._trigger_ready)
        self.assertEqual(onboard._ready_counter, 0)
        self.assertEqual(len(launched), 1)
        self.assertEqual(launched[0][1], [msg])
        self.assertTrue(launched[0][2])

        onboard._starting = True
        with patch('flexbe_onboard.flexbe_onboard.Logger.logwarn_throttle') as throttle:
            onboard._behavior_callback(msg)
        throttle.assert_called_once()
        self.assertEqual(len(launched), 1)

    def test_behavior_shutdown_handles_active_inactive_and_missing_behavior_cases(self):
        """Behavior shutdown should preempt active behaviors, return False when idle, and swallow missing-state errors."""
        preempted = []
        onboard = object.__new__(FlexbeOnboard)
        onboard._switch_lock = threading.Lock()
        onboard._running = True
        onboard._switching = False
        onboard.be = type('_Behavior', (), {'preempt': staticmethod(lambda: preempted.append(True))})()

        self.assertTrue(onboard.behavior_shutdown())
        self.assertEqual(preempted, [True])
        self.assertTrue(onboard._switching)

        idle = object.__new__(FlexbeOnboard)
        idle._switch_lock = threading.Lock()
        idle._running = False
        idle._switching = False
        idle.be = None
        self.assertFalse(idle.behavior_shutdown())

        broken = object.__new__(FlexbeOnboard)
        broken._switch_lock = threading.Lock()
        broken._running = True
        broken._switching = False
        broken.be = None
        self.assertIsNone(broken.behavior_shutdown())

    def test_onboard_shutdown_destroys_timer_and_spins_executor(self):
        """Onboard shutdown should destroy the heartbeat timer, drop feedback publisher, and drain the executor."""
        removed = []
        spins = []
        executor = type('_Executor', (), {'spin_once': staticmethod(lambda timeout_sec=0.0: spins.append(timeout_sec))})()

        onboard = object.__new__(FlexbeOnboard)
        onboard._heartbeat = object()
        onboard.destroy_timer = lambda timer: removed.append(('timer', timer))
        onboard._proxy_pub = type(
            '_ProxyPub',
            (),
            {'remove_publisher': staticmethod(lambda topic: removed.append(('publisher', topic)))},
        )()
        onboard._Node__executor_weakref = weakref.ref(executor)

        onboard.onboard_shutdown()

        self.assertEqual(removed[0][0], 'timer')
        self.assertEqual(removed[1], ('publisher', Topics._CMD_FEEDBACK_TOPIC))
        self.assertEqual(len(spins), 50)
        self.assertTrue(all(timeout == 0.001 for timeout in spins))

    def test_convert_input_data_and_convert_dict_handle_strings_and_nested_mappings(self):
        """Input conversion should skip empty keys, keep malformed strings, and recurse into nested containers."""
        onboard = object.__new__(FlexbeOnboard)

        with patch('flexbe_onboard.flexbe_onboard.Logger.loginfo') as loginfo:
            result = onboard._convert_input_data(
                ['', 'count', 'label', 'broken', 'nested'],
                ['', '3', 'plain-text', '[1,', '{"outer": {"inner": [1, 2]}}'],
            )

        self.assertEqual(result['count'], 3)
        self.assertEqual(result['label'], 'plain-text')
        self.assertEqual(result['broken'], '[1,')
        self.assertEqual(result['nested'].outer.inner, [1, 2])
        self.assertNotIn('', result)
        loginfo.assert_called_once()

    def test_cleanup_tempdir_removes_sys_path_and_ignores_missing_entry(self):
        """Tempdir cleanup should remove the temp path once and tolerate repeated calls."""
        onboard = object.__new__(FlexbeOnboard)

        with tempfile.TemporaryDirectory() as tmpdir:
            onboard._tmp_folder = tmpdir
            sys.path.append(tmpdir)

            onboard._cleanup_tempdir()
            self.assertNotIn(tmpdir, sys.path)
            self.assertFalse(os.path.exists(tmpdir))

            onboard._cleanup_tempdir()

    def test_publish_stopped_status_and_switch_leaf_filter_cover_positive_paths(self):
        """STOPPED publication should preserve the behavior id, and leaf filtering should drop containers."""
        published = []
        onboard = object.__new__(FlexbeOnboard)
        onboard._status_pub = type(
            '_StatusPub',
            (),
            {
                'publish': staticmethod(lambda msg: published.append((msg.code, msg.behavior_id))),
            },
        )()
        onboard.get_clock = lambda: type(
            '_Clock',
            (),
            {'now': staticmethod(lambda: type('_Now', (), {'to_msg': staticmethod(lambda: object())})())},
        )()

        onboard._publish_stopped_status(77)

        self.assertEqual(published, [(BEStatus.STOPPED, 77)])

        container = object.__new__(StateMachine)
        leaf = object()
        self.assertEqual(FlexbeOnboard._get_switch_leaf_states([container, leaf, None]), (leaf,))
        self.assertEqual(FlexbeOnboard._get_switch_leaf_states(None), ())

    def test_publish_stopped_status_does_not_require_subscribers(self):
        """STOPPED should still publish without consulting subscriber counts."""
        published = []
        onboard = object.__new__(FlexbeOnboard)
        onboard._status_pub = type(
            '_StatusPub',
            (),
            {
                'publish': staticmethod(lambda msg: published.append((msg.code, msg.behavior_id))),
            },
        )()
        onboard.get_clock = lambda: type(
            '_Clock',
            (),
            {'now': staticmethod(lambda: type('_Now', (), {'to_msg': staticmethod(lambda: object())})())},
        )()

        onboard._publish_stopped_status(9)

        self.assertEqual(published, [(BEStatus.STOPPED, 9)])

    def test_heartbeat_worker_publishes_behavior_status_and_resets_ready_state(self):
        """Heartbeat worker should publish active behavior status and suppress READY while running."""
        heartbeat_messages = []
        status_messages = []
        latest_status = BehaviorSync(behavior_id=77)

        onboard = object.__new__(FlexbeOnboard)
        onboard.be = type('_Behavior', (), {'get_latest_status': staticmethod(lambda: latest_status)})()
        onboard._heartbeat_pub = type(
            '_HeartbeatPub',
            (),
            {
                'get_subscription_count': staticmethod(lambda: 1),
                'publish': staticmethod(lambda msg: heartbeat_messages.append(msg)),
            },
        )()
        onboard._status_pub = type(
            '_StatusPub',
            (),
            {'publish': staticmethod(lambda msg: status_messages.append(msg))},
        )()
        onboard._ready_status = BEStatus(code=BEStatus.READY)
        onboard._idle_heartbeat = BehaviorSync()
        onboard._running = True
        onboard._switching = False
        onboard._trigger_ready = True
        onboard._ready_counter = 7

        with patch('flexbe_onboard.flexbe_onboard.Logger.check_local_enabled', self._noop):
            onboard._heartbeat_worker()

        self.assertEqual([latest_status], heartbeat_messages)
        self.assertEqual([], status_messages)
        self.assertFalse(onboard._trigger_ready)
        self.assertEqual(0, onboard._ready_counter)

    def test_heartbeat_worker_rearms_ready_after_ten_idle_cycles(self):
        """Heartbeat worker should re-arm READY publication after ten idle ticks without republishing yet."""
        heartbeat_messages = []
        status_messages = []

        onboard = object.__new__(FlexbeOnboard)
        onboard.be = None
        onboard._heartbeat_pub = type(
            '_HeartbeatPub',
            (),
            {
                'get_subscription_count': staticmethod(lambda: 1),
                'publish': staticmethod(lambda msg: heartbeat_messages.append(msg)),
            },
        )()
        onboard._status_pub = type(
            '_StatusPub',
            (),
            {'publish': staticmethod(lambda msg: status_messages.append(msg))},
        )()
        onboard._idle_heartbeat = BehaviorSync(behavior_id=-1)
        onboard._ready_status = BEStatus(code=BEStatus.READY)
        onboard._running = False
        onboard._switching = False
        onboard._trigger_ready = False
        onboard._ready_counter = 9

        with patch('flexbe_onboard.flexbe_onboard.Logger.check_local_enabled', self._noop):
            onboard._heartbeat_worker()

        self.assertEqual([-1], [msg.behavior_id for msg in heartbeat_messages])
        self.assertEqual([], status_messages)
        self.assertTrue(onboard._trigger_ready)
        self.assertEqual(10, onboard._ready_counter)

    def test_clear_imports_and_track_imports_manage_tracked_modules(self):
        """Import tracking should collect newly loaded modules and clear only those tracked entries."""
        onboard = object.__new__(FlexbeOnboard)
        onboard._tracked_imports = []
        module_name = 'codex_temp_import_module'
        sys.modules.pop(module_name, None)

        with onboard._track_imports():
            sys.modules[module_name] = object()

        self.assertIn(module_name, onboard._tracked_imports)
        onboard._clear_imports()
        self.assertEqual(onboard._tracked_imports, [])
        self.assertNotIn(module_name, sys.modules)

    def test_is_switchable_requires_matching_behavior_names(self):
        """Behavior switches should require matching behavior names and log mismatches."""
        onboard = object.__new__(FlexbeOnboard)
        onboard.be = type('_CurrentBehavior', (), {'name': 'current'})()

        matching = type('_PreparedBehavior', (), {'name': 'current'})()
        mismatched = type('_PreparedBehavior', (), {'name': 'other'})()

        self.assertTrue(onboard._is_switchable(matching))
        with patch('flexbe_onboard.flexbe_onboard.Logger.logerr') as logerr:
            self.assertFalse(onboard._is_switchable(mismatched))
        logerr.assert_called_once()


if __name__ == '__main__':
    unittest.main()
