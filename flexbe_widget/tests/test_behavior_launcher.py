#!/usr/bin/env python3

# Copyright 2026  Christopher Newport University
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


"""Focused tests for widget behavior launcher request processing."""

import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from flexbe_msgs.msg import BEStatus

import flexbe_widget.behavior_launcher as behavior_launcher
from flexbe_widget.behavior_launcher import BehaviorLauncher

from rclpy.qos import QoSDurabilityPolicy


class _FakeLogger:

    def info(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


class _FakePublisher:

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FakeClockNow:

    def to_msg(self):
        return SimpleNamespace()


class _FakeClock:

    def now(self):
        return _FakeClockNow()


class _FakeMainTime:

    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        return _FakeMainTime(self.nanoseconds - other.nanoseconds)


class _FakeMainFuture:

    def __init__(self, callback, done_after_run=True):
        self._callback = callback
        self._done = False
        self._done_after_run = done_after_run

    def run(self):
        if self._callback is not None:
            self._callback()
        self._done = self._done_after_run

    def done(self):
        return self._done


class _FakeMainExecutor:

    def __init__(self, launcher, future_done_after_run=True, spin_exception=None):
        self._launcher = launcher
        self._future_done_after_run = future_done_after_run
        self._spin_exception = spin_exception
        self.added_nodes = []
        self.spin_calls = 0
        self.spin_once_calls = []
        self.future = None
        self.spin_until_future_complete_calls = []

    def add_node(self, node):
        self.added_nodes.append(node)

    def spin_once(self, timeout_sec=None):
        self.spin_once_calls.append(timeout_sec)
        if self._launcher._last_onboard_heartbeat is None:
            self._launcher._last_onboard_heartbeat = object()
        if not self._launcher._ready_event.is_set():
            self._launcher._ready_event.set()

    def create_task(self, callback):
        self.future = _FakeMainFuture(callback, done_after_run=self._future_done_after_run)
        return self.future

    def spin_until_future_complete(self, future, timeout_sec=None):
        self.spin_until_future_complete_calls.append(timeout_sec)
        future.run()

    def spin(self):
        self.spin_calls += 1
        if self._spin_exception is not None:
            raise self._spin_exception


class _FakeBehaviorLibrary:

    def __init__(self, behavior_entry=None, source_path=None, tmp_path=None):
        self._behavior_entry = behavior_entry
        self._source_path = source_path
        self._tmp_path = tmp_path

    def find_behavior(self, _name):
        if self._behavior_entry is None:
            return None, None
        return 23, self._behavior_entry

    def get_sourcecode_filepath(self, _be_key, add_tmp=False):
        if (add_tmp and self._tmp_path is None) or (not add_tmp and self._source_path is None):
            raise FileNotFoundError('missing behavior source')
        return self._tmp_path if add_tmp else self._source_path


class _FakeStateMapPub(_FakePublisher):
    pass


class _FakeMainLauncher:

    def __init__(self, destroy_exception=None):
        self._last_onboard_heartbeat = None
        self._last_heartbeat_msg = SimpleNamespace(behavior_id=0, current_state_checksums=[])
        self._ready_event = threading.Event()
        self.requests = []
        self.destroy_calls = 0
        self._clock_reads = 0
        self._destroy_exception = destroy_exception

    def get_clock(self):
        def now():
            self._clock_reads += 1
            return _FakeMainTime(self._clock_reads * 10**9)
        return SimpleNamespace(now=now)

    def _request_callback(self, request):
        self.requests.append(request)

    def destroy_node(self):
        self.destroy_calls += 1
        if self._destroy_exception is not None:
            raise self._destroy_exception


class TestBehaviorLauncher(unittest.TestCase):
    """Test launcher request handling without ROS node startup."""

    def _make_launcher(self, behavior_lib):
        launcher = object.__new__(BehaviorLauncher)
        launcher._behavior_lib = behavior_lib
        launcher._ready_event = threading.Event()
        launcher._ready_event.set()
        launcher._command_feedback_pub = _FakePublisher()
        launcher._status_pub = _FakePublisher()
        launcher._mirror_pub = _FakePublisher()
        launcher._pub = _FakePublisher()
        launcher._state_map_pub = _FakeStateMapPub()
        launcher.get_logger = lambda: _FakeLogger()
        launcher.get_clock = lambda: _FakeClock()
        return launcher

    def test_process_request_reports_error_for_unknown_behavior(self):
        """Report launcher-local feedback when the requested behavior cannot be found."""
        launcher = self._make_launcher(_FakeBehaviorLibrary())
        msg = SimpleNamespace(behavior_name='missing', autonomy_level=255,
                              arg_keys=[], arg_values=[], structure=[])

        launcher._process_request(msg)

        self.assertEqual([], launcher._status_pub.messages)
        self.assertEqual(1, len(launcher._command_feedback_pub.messages))
        feedback = launcher._command_feedback_pub.messages[0]
        self.assertEqual('launch', feedback.command)
        self.assertEqual(['blocked', 'behavior_not_found'], list(feedback.args))
        self.assertEqual([], launcher._pub.messages)

    def test_request_callback_reports_launch_feedback_when_launcher_is_not_ready(self):
        """Publish launcher-local feedback instead of a synthetic onboard error."""
        launcher = self._make_launcher(_FakeBehaviorLibrary())
        launcher._ready_event.clear()
        processed_requests = []
        launcher._process_request = lambda msg: processed_requests.append(msg)
        msg = SimpleNamespace(behavior_name='demo', autonomy_level=255,
                              arg_keys=[], arg_values=[], structure=[])

        with patch('flexbe_widget.behavior_launcher.Logger.logerr'):
            launcher._request_callback(msg)

        self.assertEqual([], processed_requests)
        self.assertEqual([], launcher._status_pub.messages)
        self.assertEqual(1, len(launcher._command_feedback_pub.messages))
        feedback = launcher._command_feedback_pub.messages[0]
        self.assertEqual('launch', feedback.command)
        self.assertEqual(['blocked', 'not_ready'], list(feedback.args))

    def test_process_request_reports_launch_feedback_for_arg_mismatch(self):
        """Reject malformed argument arrays before publishing synthetic onboard status."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            source_path.write_text('class Demo: pass\n', encoding='utf-8')
            launcher = self._make_launcher(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(source_path.with_name('demo_tmp.py')),
            ))
            msg = SimpleNamespace(
                behavior_name='demo',
                autonomy_level=255,
                arg_keys=['speed'],
                arg_values=[],
                structure=[],
            )

            launcher._process_request(msg)

            self.assertEqual([], launcher._status_pub.messages)
            self.assertEqual([], launcher._pub.messages)
            self.assertEqual(1, len(launcher._command_feedback_pub.messages))
            feedback = launcher._command_feedback_pub.messages[0]
            self.assertEqual(['blocked', 'arg_mismatch'], list(feedback.args))

    def test_process_request_reports_launch_feedback_for_invalid_structure(self):
        """Reject malformed structure data locally instead of raising or publishing BE status."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            source_path.write_text('class Demo: pass\n', encoding='utf-8')
            launcher = self._make_launcher(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(source_path.with_name('demo_tmp.py')),
            ))
            msg = SimpleNamespace(
                behavior_name='demo',
                autonomy_level=255,
                arg_keys=[],
                arg_values=[],
                structure=[SimpleNamespace()],
            )

            launcher._process_request(msg)

            self.assertEqual([], launcher._status_pub.messages)
            self.assertEqual([], launcher._pub.messages)
            self.assertEqual(1, len(launcher._command_feedback_pub.messages))
            feedback = launcher._command_feedback_pub.messages[0]
            self.assertEqual(['blocked', 'invalid_structure'], list(feedback.args))

    def test_process_request_reports_launch_feedback_for_missing_package(self):
        """Reject missing source packages locally instead of publishing synthetic onboard errors."""
        launcher = self._make_launcher(_FakeBehaviorLibrary(
            behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
            source_path=None,
            tmp_path=None,
        ))
        msg = SimpleNamespace(
            behavior_name='demo',
            autonomy_level=255,
            arg_keys=[],
            arg_values=[],
            structure=[],
        )

        launcher._process_request(msg)

        self.assertEqual([], launcher._status_pub.messages)
        self.assertEqual([], launcher._pub.messages)
        self.assertEqual(1, len(launcher._command_feedback_pub.messages))
        feedback = launcher._command_feedback_pub.messages[0]
        self.assertEqual(['blocked', 'package_not_found'], list(feedback.args))

    def test_external_error_clears_ready_event_until_ready_arrives(self):
        """Keep requests blocked on terminal statuses until onboard republishes READY."""
        launcher = self._make_launcher(_FakeBehaviorLibrary())
        launcher._ready_event.set()

        launcher._status_callback(SimpleNamespace(code=BEStatus.ERROR, args=[]))

        self.assertFalse(launcher._ready_event.is_set())

    def test_process_request_expands_yaml_arguments(self):
        """Expand /YAML arguments into the outgoing behavior selection payload."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            source_path.write_text('class Demo: pass\n', encoding='utf-8')
            yaml_path = Path(temp_dir) / 'params.yaml'
            yaml_path.write_text('outer:\n  speed: fast\n', encoding='utf-8')
            launcher = self._make_launcher(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(source_path.with_name('demo_tmp.py')),
            ))
            msg = SimpleNamespace(
                behavior_name='demo',
                autonomy_level=255,
                arg_keys=['/YAML:/config'],
                arg_values=[f'{yaml_path}:outer'],
                structure=[],
            )

            launcher._process_request(msg)

            self.assertEqual(1, len(launcher._pub.messages))
            selection = launcher._pub.messages[0]
            self.assertEqual(['/config'], list(selection.arg_keys))
            self.assertIn('speed: fast', selection.arg_values[0])

    def test_process_request_falls_back_to_direct_args_for_unsafe_yaml_tags(self):
        """Unsafe YAML tags should be rejected and preserve the original argument arrays."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            source_path.write_text('class Demo: pass\n', encoding='utf-8')
            yaml_path = Path(temp_dir) / 'params.yaml'
            yaml_path.write_text('!!python/object/apply:os.system ["echo blocked"]\n', encoding='utf-8')
            launcher = self._make_launcher(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(source_path.with_name('demo_tmp.py')),
            ))
            msg = SimpleNamespace(
                behavior_name='demo',
                autonomy_level=255,
                arg_keys=['/YAML:/config'],
                arg_values=[f'{yaml_path}:outer'],
                structure=[],
            )

            launcher._process_request(msg)

            self.assertEqual(1, len(launcher._pub.messages))
            selection = launcher._pub.messages[0]
            self.assertEqual(list(msg.arg_keys), list(selection.arg_keys))
            self.assertEqual(list(msg.arg_values), list(selection.arg_values))

    def test_process_request_skips_mirror_publish_for_detached_autonomy(self):
        """Do not publish mirror structure when autonomy_level is the detached sentinel."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            source_path.write_text('class Demo: pass\n', encoding='utf-8')
            launcher = self._make_launcher(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(source_path.with_name('demo_tmp.py')),
            ))
            msg = SimpleNamespace(
                behavior_name='demo',
                autonomy_level=255,
                arg_keys=[],
                arg_values=[],
                structure=[],
            )

            launcher._process_request(msg)

            self.assertEqual([], launcher._mirror_pub.messages)
            self.assertEqual(1, len(launcher._pub.messages))

    def test_process_request_publishes_mirror_structure_for_supervised_launch(self):
        """Publish mirror structure when launching with a real autonomy level."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            source_path.write_text('class Demo: pass\n', encoding='utf-8')
            launcher = self._make_launcher(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(source_path.with_name('demo_tmp.py')),
            ))
            msg = SimpleNamespace(
                behavior_name='demo',
                autonomy_level=2,
                arg_keys=[],
                arg_values=[],
                structure=[],
            )

            launcher._process_request(msg)

            self.assertEqual(1, len(launcher._mirror_pub.messages))
            self.assertEqual(1, len(launcher._pub.messages))

    def test_heartbeat_worker_tracks_missing_and_stale_onboard_heartbeats(self):
        """Heartbeat timer should warn when no onboard heartbeat was seen or when it goes stale."""
        launcher = self._make_launcher(_FakeBehaviorLibrary())
        warnings = []

        class _FakeTime:

            def __init__(self, seconds):
                self._seconds = seconds

            def seconds_nanoseconds(self):
                return (self._seconds, 0)

            def __sub__(self, other):
                return SimpleNamespace(nanoseconds=(self._seconds - other._seconds) * 10**9)

        heartbeat_times = iter([_FakeTime(1), _FakeTime(4)])
        launcher._heartbeat_pub = _FakePublisher()
        launcher.get_clock = lambda: SimpleNamespace(
            now=lambda: next(heartbeat_times)
        )
        launcher.get_logger = lambda: SimpleNamespace(
            info=lambda *_args, **_kwargs: None,
            warning=lambda msg: warnings.append(msg),
            error=lambda *_args, **_kwargs: None,
        )

        launcher._last_onboard_heartbeat = None
        launcher.heartbeat_timer_callback()

        stale_now = _FakeTime(10)
        stale_before = _FakeTime(7)
        launcher.get_clock = lambda: SimpleNamespace(now=lambda: stale_now)
        launcher._last_onboard_heartbeat = stale_before
        launcher.heartbeat_timer_callback()

        self.assertEqual(2, len(launcher._heartbeat_pub.messages))
        self.assertEqual(2, len(warnings))
        self.assertIsNone(launcher._last_onboard_heartbeat)

    def test_onboard_heartbeat_callback_keeps_fresh_heartbeat_and_timer_only_warns_when_stale(self):
        """Heartbeat callback should refresh launcher state until timer later marks it stale."""
        launcher = self._make_launcher(_FakeBehaviorLibrary())
        warnings = []

        class _FakeTime:

            def __init__(self, seconds):
                self._seconds = seconds

            def seconds_nanoseconds(self):
                return (self._seconds, 0)

            def __sub__(self, other):
                return SimpleNamespace(nanoseconds=(self._seconds - other._seconds) * 10**9)

        times = iter([_FakeTime(1), _FakeTime(2), _FakeTime(3), _FakeTime(5), _FakeTime(6)])
        launcher._heartbeat_pub = _FakePublisher()
        launcher.get_clock = lambda: SimpleNamespace(now=lambda: next(times))
        launcher.get_logger = lambda: SimpleNamespace(
            info=lambda *_args, **_kwargs: None,
            warning=lambda msg: warnings.append(msg),
            error=lambda *_args, **_kwargs: None,
        )
        msg = SimpleNamespace(behavior_id=17)

        launcher._onboard_heartbeat_callback(msg)
        launcher.heartbeat_timer_callback()

        self.assertIs(msg, launcher._last_heartbeat_msg)
        self.assertEqual([], warnings)

        launcher.heartbeat_timer_callback()

        self.assertEqual(2, len(launcher._heartbeat_pub.messages))
        self.assertEqual(1, len(warnings))
        self.assertIsNone(launcher._last_onboard_heartbeat)

    def test_startup_probe_callback_cancels_timer_and_logs_once(self):
        """Startup probe should cancel itself after proving timer callbacks are serviced."""
        infos = []
        launcher = self._make_launcher(_FakeBehaviorLibrary())
        launcher._startup_probe_timer = SimpleNamespace(cancel=lambda: infos.append('cancel'))
        launcher.get_logger = lambda: SimpleNamespace(
            info=lambda msg: infos.append(msg),
            warning=lambda *_args, **_kwargs: None,
            error=lambda *_args, **_kwargs: None,
        )

        launcher._startup_probe_callback()

        self.assertEqual(
            [
                'cancel',
                'Behavior launcher active; publishers are initialized '
                'and executor is servicing timer callbacks.',
            ],
            infos,
        )

    def test_launch_feedback_helpers_publish_command_feedback_and_log_errors(self):
        """Launch feedback helpers should publish blocked feedback and log local rejection reasons."""
        launcher = self._make_launcher(_FakeBehaviorLibrary())
        errors = []
        launcher.get_logger = lambda: SimpleNamespace(
            info=lambda *_args, **_kwargs: None,
            warning=lambda *_args, **_kwargs: None,
            error=lambda msg: errors.append(msg),
        )

        launcher._publish_launch_feedback('blocked', 'reason')
        launcher._reject_launch('bad_request', 'bad launch')

        self.assertEqual(2, len(launcher._command_feedback_pub.messages))
        self.assertEqual(['blocked', 'reason'], list(launcher._command_feedback_pub.messages[0].args))
        self.assertEqual(['blocked', 'bad_request'], list(launcher._command_feedback_pub.messages[1].args))
        self.assertEqual(['bad launch'], errors)

    def test_process_request_falls_back_to_direct_args_when_yaml_expansion_fails(self):
        """YAML expansion failures should preserve the original argument arrays."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            source_path.write_text('class Demo: pass\n', encoding='utf-8')
            launcher = self._make_launcher(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(source_path.with_name('demo_tmp.py')),
            ))
            msg = SimpleNamespace(
                behavior_name='demo',
                autonomy_level=255,
                arg_keys=['/YAML:/config', 'plain'],
                arg_values=['/does/not/exist.yaml:outer', 'value'],
                structure=[],
            )

            launcher._process_request(msg)

            self.assertEqual(1, len(launcher._pub.messages))
            selection = launcher._pub.messages[0]
            self.assertEqual(list(msg.arg_keys), list(selection.arg_keys))
            self.assertEqual(list(msg.arg_values), list(selection.arg_values))

    def test_process_request_tolerates_state_map_publish_failures_and_still_launches(self):
        """Launcher should still publish the behavior selection if state-map publication fails."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            source_path.write_text('class Demo: pass\n', encoding='utf-8')
            launcher = self._make_launcher(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(source_path.with_name('demo_tmp.py')),
            ))
            launcher._state_map_pub = SimpleNamespace(
                publish=lambda _msg: (_ for _ in ()).throw(RuntimeError('state map boom'))
            )
            msg = SimpleNamespace(
                behavior_name='demo',
                autonomy_level=2,
                arg_keys=[],
                arg_values=[],
                structure=[SimpleNamespace(path='/root', state_id=1, outcomes=[], autonomy=[])],
            )

            launcher._process_request(msg)

            self.assertEqual(1, len(launcher._pub.messages))
            self.assertEqual(1, len(launcher._mirror_pub.messages))

    def test_process_request_builds_modifications_when_behavior_source_changes(self):
        """Launcher should publish modifications when the temp behavior source differs."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            tmp_path = Path(temp_dir) / 'demo_tmp.py'
            source_path.write_text('alpha\nbeta\ngamma\n', encoding='utf-8')
            tmp_path.write_text('alpha\nbeta\nold\n', encoding='utf-8')
            launcher = self._make_launcher(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(tmp_path),
            ))
            msg = SimpleNamespace(
                behavior_name='demo',
                autonomy_level=2,
                arg_keys=[],
                arg_values=[],
                structure=[SimpleNamespace(path='/root', state_id=17, outcomes=[], autonomy=[])],
            )

            launcher._process_request(msg)

            self.assertEqual(1, len(launcher._pub.messages))
            selection = launcher._pub.messages[0]
            self.assertGreaterEqual(len(selection.modifications), 1)
            self.assertTrue(any(mod.new_content for mod in selection.modifications))
            self.assertEqual(1, len(launcher._state_map_pub.messages))
            self.assertEqual(1, len(launcher._mirror_pub.messages))
            self.assertFalse(launcher._ready_event.is_set())

    def test_version_callback_warns_only_for_outdated_ui_versions(self):
        """Version callback should only warn when the UI version is below the launcher minimum."""
        self.assertEqual(10203, BehaviorLauncher._parse_version('1.2.3'))

        launcher = self._make_launcher(_FakeBehaviorLibrary())
        with patch('flexbe_widget.behavior_launcher.Logger.logwarn') as logwarn:
            launcher._version_callback(SimpleNamespace(data='0.0.1'))
            launcher._version_callback(SimpleNamespace(data='999.0.0'))

        logwarn.assert_called_once()

    def test_ui_version_qos_is_transient_local(self):
        """UI version subscription should receive latched versions from the WebUI."""
        qos = BehaviorLauncher._latched_qos(depth=1)

        self.assertEqual(1, qos.depth)
        self.assertEqual(QoSDurabilityPolicy.TRANSIENT_LOCAL, qos.durability)

    def test_behavior_launcher_main_spins_cleanly_without_behavior_request(self):
        """CLI main should initialize ROS and spin even when no behavior is requested."""
        launcher = _FakeMainLauncher()
        executor = _FakeMainExecutor(launcher)

        with patch('flexbe_widget.behavior_launcher.BehaviorLauncher', return_value=launcher), \
                patch('flexbe_widget.behavior_launcher.rclpy.executors.SingleThreadedExecutor',
                      return_value=executor), \
                patch('flexbe_widget.behavior_launcher.rclpy.init') as init_mock, \
                patch('flexbe_widget.behavior_launcher.rclpy.try_shutdown') as try_shutdown_mock, \
                patch('flexbe_widget.behavior_launcher.sys.argv', ['behavior_launcher']), \
                patch('builtins.print') as print_mock:
            behavior_launcher.behavior_launcher_main()

        init_mock.assert_called_once_with(
            args=[],
            signal_handler_options=behavior_launcher.rclpy.signals.SignalHandlerOptions.NO,
        )
        self.assertEqual([launcher], executor.added_nodes)
        self.assertEqual(1, executor.spin_calls)
        self.assertEqual(1, launcher.destroy_calls)
        try_shutdown_mock.assert_called_once_with()
        printed = '\n'.join(call.args[0] for call in print_mock.call_args_list if call.args)
        self.assertIn("Behavior launcher with behavior'' autonomy=255 auto_start=False", printed)
        self.assertIn('Start behavior_launcher spinner', printed)

    def test_behavior_launcher_main_autostart_waits_then_submits_initial_request(self):
        """CLI main should build and submit the initial request after heartbeat/ready conditions are met."""
        launcher = _FakeMainLauncher()
        executor = _FakeMainExecutor(launcher)

        with patch('flexbe_widget.behavior_launcher.BehaviorLauncher', return_value=launcher), \
                patch('flexbe_widget.behavior_launcher.rclpy.executors.SingleThreadedExecutor',
                      return_value=executor), \
                patch('flexbe_widget.behavior_launcher.rclpy.init'), \
                patch('flexbe_widget.behavior_launcher.rclpy.try_shutdown'), \
                patch('flexbe_widget.behavior_launcher.Logger.info'), \
                patch('flexbe_widget.behavior_launcher.Logger.warning'), \
                patch('flexbe_widget.behavior_launcher.Logger.error'), \
                patch('flexbe_widget.behavior_launcher.sys.argv',
                      ['behavior_launcher', '--behavior', 'Demo', '--autonomy', '3', '--autostart',
                       '--ros-args', '-r', '__ns:=/demo']), \
                patch('builtins.print'):
            behavior_launcher.behavior_launcher_main()

        self.assertEqual([launcher], executor.added_nodes)
        self.assertEqual([10.0], executor.spin_until_future_complete_calls)
        self.assertEqual(1, executor.spin_calls)
        self.assertEqual(1, len(launcher.requests))
        request = launcher.requests[0]
        self.assertEqual('Demo', request.behavior_name)
        self.assertEqual(3, request.autonomy_level)
        self.assertEqual([], list(request.arg_keys))
        self.assertEqual([], list(request.arg_values))

    def test_behavior_launcher_main_collects_behavior_arguments_and_ignores_internal_keys(self):
        """CLI main should forward custom behavior args and skip internal roslaunch keys."""
        launcher = _FakeMainLauncher()
        executor = _FakeMainExecutor(launcher)

        with patch('flexbe_widget.behavior_launcher.BehaviorLauncher', return_value=launcher), \
                patch('flexbe_widget.behavior_launcher.rclpy.executors.SingleThreadedExecutor',
                      return_value=executor), \
                patch('argparse.ArgumentParser.parse_args',
                      return_value=SimpleNamespace(behavior='Demo', autonomy=255, autostart=False)), \
                patch('flexbe_widget.behavior_launcher.rclpy.init'), \
                patch('flexbe_widget.behavior_launcher.rclpy.try_shutdown'), \
                patch('flexbe_widget.behavior_launcher.Logger.info'), \
                patch('flexbe_widget.behavior_launcher.Logger.warning'), \
                patch('flexbe_widget.behavior_launcher.Logger.error'), \
                patch('flexbe_widget.behavior_launcher.sys.argv',
                      ['behavior_launcher', '--behavior', 'Demo', 'speed:=fast',
                       '__node:=ignored', '__log:=skipme']), \
                patch('builtins.print'):
            behavior_launcher.behavior_launcher_main()

        self.assertEqual(1, len(launcher.requests))
        request = launcher.requests[0]
        self.assertEqual(['/speed'], list(request.arg_keys))
        self.assertEqual(['fast'], list(request.arg_values))

    def test_behavior_launcher_main_reports_initial_request_timeout(self):
        """CLI main should log an error if the initial future never completes."""
        launcher = _FakeMainLauncher()
        executor = _FakeMainExecutor(launcher, future_done_after_run=False)

        with patch('flexbe_widget.behavior_launcher.BehaviorLauncher', return_value=launcher), \
                patch('flexbe_widget.behavior_launcher.rclpy.executors.SingleThreadedExecutor',
                      return_value=executor), \
                patch('flexbe_widget.behavior_launcher.rclpy.init'), \
                patch('flexbe_widget.behavior_launcher.rclpy.try_shutdown'), \
                patch('flexbe_widget.behavior_launcher.Logger.info'), \
                patch('flexbe_widget.behavior_launcher.Logger.warning'), \
                patch('flexbe_widget.behavior_launcher.Logger.error') as error_log, \
                patch('flexbe_widget.behavior_launcher.sys.argv',
                      ['behavior_launcher', '--behavior', 'Demo']), \
                patch('builtins.print'):
            behavior_launcher.behavior_launcher_main()

        error_log.assert_called_once()
        self.assertIn('timed out', error_log.call_args.args[0])

    def test_behavior_launcher_main_parse_error_prints_help_and_exits(self):
        """Argument parse failures should print help and exit with a nonzero code."""
        with patch('argparse.ArgumentParser.parse_args', side_effect=ValueError('bad args')), \
                patch('argparse.ArgumentParser.print_help') as print_help, \
                patch('flexbe_widget.behavior_launcher.sys.argv', ['behavior_launcher', '--broken']), \
                patch('flexbe_widget.behavior_launcher.sys.exit', side_effect=SystemExit(-1)) as exit_mock, \
                patch('builtins.print') as print_mock:
            with self.assertRaises(SystemExit):
                behavior_launcher.behavior_launcher_main()

        print_help.assert_called_once()
        exit_mock.assert_called_once_with(-1)
        self.assertEqual('bad args', str(print_mock.call_args.args[0]))

    def test_behavior_launcher_main_handles_keyboard_interrupt_and_shutdown_exceptions(self):
        """Keyboard interrupt and shutdown exceptions should be reported without crashing."""
        launcher = _FakeMainLauncher(destroy_exception=RuntimeError('destroy boom'))
        executor = _FakeMainExecutor(launcher, spin_exception=KeyboardInterrupt())

        with patch('flexbe_widget.behavior_launcher.BehaviorLauncher', return_value=launcher), \
                patch('flexbe_widget.behavior_launcher.rclpy.executors.SingleThreadedExecutor',
                      return_value=executor), \
                patch('flexbe_widget.behavior_launcher.rclpy.init'), \
                patch('flexbe_widget.behavior_launcher.rclpy.try_shutdown',
                      side_effect=RuntimeError('shutdown boom')), \
                patch('traceback.format_exc', return_value='traceback%%body'), \
                patch('flexbe_widget.behavior_launcher.sys.argv', ['behavior_launcher']), \
                patch('builtins.print') as print_mock:
            behavior_launcher.behavior_launcher_main()

        printed = '\n'.join(call.args[0] for call in print_mock.call_args_list if call.args)
        self.assertIn('Keyboard interrupt request', printed)
        self.assertIn('Exception from destroy behavior launcher node', printed)
        self.assertIn('Exception from rclpy.try_shutdown for behavior launcher', printed)


if __name__ == '__main__':
    unittest.main()
