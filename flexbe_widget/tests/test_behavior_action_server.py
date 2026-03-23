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


"""Focused tests for widget behavior action server request and status handling."""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from flexbe_msgs.msg import BEStatus

from flexbe_widget.behavior_action_server import BehaviorActionServer

from rclpy.action import GoalResponse


class _FakeLogger:

    def __init__(self):
        self.warnings = []
        self.errors = []
        self.infos = []

    def info(self, *_args, **_kwargs):
        self.infos.append((_args, _kwargs))

    def warning(self, *_args, **_kwargs):
        self.warnings.append((_args, _kwargs))

    def error(self, *_args, **_kwargs):
        self.errors.append((_args, _kwargs))

    def loginfo(self, *_args, **_kwargs):
        pass


class _FakeNode:

    def __init__(self):
        self._logger = _FakeLogger()

    def get_logger(self):
        return self._logger


class _FakePublisher:

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FakeBehaviorLibrary:

    def __init__(self, behavior_entry=None, source_path=None, tmp_path=None, source_error=None):
        self._behavior_entry = behavior_entry
        self._source_path = source_path
        self._tmp_path = tmp_path
        self._source_error = source_error

    def find_behavior(self, name):
        if self._behavior_entry is None:
            return None, None
        return 17, self._behavior_entry

    def get_sourcecode_filepath(self, _be_key, add_tmp=False):
        if self._source_error is not None:
            raise self._source_error
        return self._tmp_path if add_tmp else self._source_path


class _FakeGoal:

    def __init__(self, behavior_name='demo', arg_keys=None, arg_values=None, input_keys=None, input_values=None):
        self.is_active = True
        self.canceled_count = 0
        self.aborted_count = 0
        self.succeeded_count = 0
        self.executed_count = 0
        self.request = SimpleNamespace(
            behavior_name=behavior_name,
            arg_keys=list(arg_keys or []),
            arg_values=list(arg_values or []),
            input_keys=list(input_keys or []),
            input_values=list(input_values or []),
        )

    def canceled(self):
        self.canceled_count += 1
        self.is_active = False

    def abort(self):
        self.aborted_count += 1
        self.is_active = False

    def succeed(self):
        self.succeeded_count += 1
        self.is_active = False

    def execute(self):
        self.executed_count += 1

    def publish_feedback(self, _feedback):
        pass


class TestBehaviorActionServer(unittest.TestCase):
    """Test action-server request construction and terminal status handling."""

    def _make_server(self, behavior_lib):
        server = object.__new__(BehaviorActionServer)
        server._node = _FakeNode()
        server._behavior_lib = behavior_lib
        server._pub = _FakePublisher()
        server._preempt_pub = _FakePublisher()
        server._behavior_started = False
        server._preempt_requested = False
        server._current_goal = None
        server._requested_behavior_id = None
        server._current_state = None
        server._active_behavior_id = None
        server.running = False
        server.outcome = ''
        return server

    def test_goal_cb_rejects_malformed_argument_arrays(self):
        """Reject malformed goal payloads before publishing a behavior selection."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            source_path.write_text('class Demo: pass\n', encoding='utf-8')
            server = self._make_server(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(source_path.with_name('demo_tmp.py')),
            ))
            goal = _FakeGoal(arg_keys=['a'], arg_values=[])

            server._goal_cb(goal)

            self.assertEqual(1, goal.aborted_count)
            self.assertEqual([], server._pub.messages)

    def test_goal_cb_aborts_unknown_behavior_requests(self):
        """Abort invalid start requests instead of reporting them as client cancels."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal(behavior_name='missing')

        server._goal_cb(goal)

        self.assertEqual(1, goal.aborted_count)
        self.assertEqual(0, goal.canceled_count)
        self.assertEqual([], server._pub.messages)

    def test_goal_cb_publishes_behavior_selection_for_valid_goal(self):
        """Publish a selection containing the requested args and inputs."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            source_path.write_text('class Demo: pass\n', encoding='utf-8')
            server = self._make_server(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(source_path.with_name('demo_tmp.py')),
            ))
            goal = _FakeGoal(arg_keys=['speed'], arg_values=['fast'],
                             input_keys=['target'], input_values=['dock'])

            server._goal_cb(goal)

            self.assertEqual(1, goal.executed_count)
            self.assertEqual(1, len(server._pub.messages))
            selection = server._pub.messages[0]
            self.assertEqual(17, selection.behavior_key)
            self.assertEqual(selection.behavior_id, server._requested_behavior_id)
            self.assertEqual(['speed'], list(selection.arg_keys))
            self.assertEqual(['fast'], list(selection.arg_values))
            self.assertEqual(['target'], list(selection.input_keys))
            self.assertEqual(['dock'], list(selection.input_values))

    def test_goal_cb_rejects_malformed_input_arrays(self):
        """Reject malformed input arrays before publishing a behavior selection."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            source_path.write_text('class Demo: pass\n', encoding='utf-8')
            server = self._make_server(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(source_path.with_name('demo_tmp.py')),
            ))
            goal = _FakeGoal(input_keys=['dock'], input_values=[])

            server._goal_cb(goal)

            self.assertEqual(1, goal.aborted_count)
            self.assertEqual([], server._pub.messages)

    def test_goal_cb_expands_file_arguments_and_builds_modifications(self):
        """Expand file arguments and attach source modifications when the tmp file differs."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            tmp_path = Path(temp_dir) / 'demo_tmp.py'
            yaml_path = Path(temp_dir) / 'params.yaml'
            source_path.write_text('alpha\nnew\n', encoding='utf-8')
            tmp_path.write_text('alpha\nold\n', encoding='utf-8')
            yaml_path.write_text('outer:\n  speed: fast\n', encoding='utf-8')
            server = self._make_server(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(tmp_path),
            ))
            goal = _FakeGoal(arg_keys=['config'],
                             arg_values=[f'file://{yaml_path}:outer'])

            server._goal_cb(goal)

            self.assertEqual(1, goal.executed_count)
            self.assertEqual(1, len(server._pub.messages))
            selection = server._pub.messages[0]
            self.assertEqual(['config'], list(selection.arg_keys))
            self.assertIn('speed: fast', selection.arg_values[0])
            self.assertGreaterEqual(len(selection.modifications), 1)

    def test_goal_cb_falls_back_to_direct_arguments_for_unsafe_yaml_tags(self):
        """Unsafe YAML tags should be rejected and preserve the original argument arrays."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            yaml_path = Path(temp_dir) / 'params.yaml'
            source_path.write_text('class Demo: pass\n', encoding='utf-8')
            yaml_path.write_text('!!python/object/apply:os.system ["echo blocked"]\n', encoding='utf-8')
            server = self._make_server(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(source_path.with_name('demo_tmp.py')),
            ))
            goal = _FakeGoal(arg_keys=['config'],
                             arg_values=[f'file://{yaml_path}:outer'])

            server._goal_cb(goal)

            self.assertEqual(1, len(server._pub.messages))
            selection = server._pub.messages[0]
            self.assertEqual(list(goal.request.arg_keys), list(selection.arg_keys))
            self.assertEqual(list(goal.request.arg_values), list(selection.arg_values))

    def test_goal_cb_falls_back_to_direct_arguments_when_file_expansion_fails(self):
        """Use the original argument arrays if file or YAML expansion fails."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source_path = Path(temp_dir) / 'demo.py'
            source_path.write_text('class Demo: pass\n', encoding='utf-8')
            server = self._make_server(_FakeBehaviorLibrary(
                behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
                source_path=str(source_path),
                tmp_path=str(source_path.with_name('demo_tmp.py')),
            ))
            goal = _FakeGoal(arg_keys=['config', 'plain'],
                             arg_values=['file:///does/not/exist.yaml:outer', 'value'])

            server._goal_cb(goal)

            self.assertEqual(1, len(server._pub.messages))
            selection = server._pub.messages[0]
            self.assertEqual(list(goal.request.arg_keys), list(selection.arg_keys))
            self.assertEqual(list(goal.request.arg_values), list(selection.arg_values))

    def test_goal_cb_aborts_setup_exceptions_and_unwedges_server(self):
        """Abort accepted goals if setup raises so later requests are not blocked forever."""
        server = self._make_server(_FakeBehaviorLibrary(
            behavior_entry={'name': 'demo', 'package': 'demo_pkg'},
            source_error=RuntimeError('boom'),
        ))
        goal = _FakeGoal()

        server._goal_cb(goal)

        self.assertEqual(1, goal.aborted_count)
        self.assertFalse(server.running)
        self.assertEqual([], server._pub.messages)
        self.assertEqual(GoalResponse.ACCEPT, server._goal_request_cb(SimpleNamespace()))

    def test_goal_request_cb_rejects_when_behavior_is_running(self):
        """Reject new goals before acceptance while a behavior is already active."""
        server = self._make_server(_FakeBehaviorLibrary())
        server.running = True

        response = server._goal_request_cb(SimpleNamespace())

        self.assertEqual(GoalResponse.REJECT, response)

    def test_goal_request_cb_rejects_when_preempt_is_pending(self):
        """Reject new goals before acceptance while a pending preempt is still being resolved."""
        server = self._make_server(_FakeBehaviorLibrary())
        server._preempt_requested = True

        response = server._goal_request_cb(SimpleNamespace())

        self.assertEqual(GoalResponse.REJECT, response)

    def test_goal_request_cb_accepts_when_idle(self):
        """Accept new goals when there is no active execution or pending preempt."""
        server = self._make_server(_FakeBehaviorLibrary())

        response = server._goal_request_cb(SimpleNamespace())

        self.assertEqual(GoalResponse.ACCEPT, response)

    def test_cancel_before_start_does_not_block_future_goals(self):
        """Clear the pending-preempt latch when a goal is canceled before startup."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()

        server._preempt_cb(goal)
        server._goal_cb(goal)

        self.assertEqual(1, goal.canceled_count)
        self.assertFalse(server._preempt_requested)
        self.assertFalse(server.running)
        self.assertEqual(GoalResponse.ACCEPT, server._goal_request_cb(SimpleNamespace()))

    def test_status_cb_started_honors_pending_preempt(self):
        """Preempt requests queued before STARTED should be forwarded once execution begins."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()
        server._current_goal = goal
        server._preempt_requested = True
        server._requested_behavior_id = 51
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.STARTED, behavior_id=51, args=[]))

        self.assertTrue(server._behavior_started)
        self.assertEqual(51, server._active_behavior_id)
        self.assertIsNone(server._requested_behavior_id)
        self.assertEqual(1, len(server._preempt_pub.messages))

    def test_status_cb_finishes_active_goal(self):
        """Mark the goal successful when the active behavior finishes."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()
        server._current_goal = goal
        server._behavior_started = True
        server._active_behavior_id = 51
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.FINISHED, behavior_id=51, args=['done']))

        self.assertEqual(1, goal.succeeded_count)
        self.assertEqual('success', server.outcome)
        self.assertFalse(server.running)

    def test_status_cb_marks_preempted_goal_canceled(self):
        """Report preempted behaviors as canceled to action clients."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()
        server._current_goal = goal
        server._behavior_started = True
        server._active_behavior_id = 51
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.FINISHED, behavior_id=51, args=['preempted']))

        self.assertEqual(1, goal.canceled_count)
        self.assertEqual(0, goal.succeeded_count)
        self.assertEqual('preempted', server.outcome)
        self.assertFalse(server.running)

    def test_status_cb_fails_active_goal(self):
        """Abort the goal when the active behavior reports FAILED."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()
        server._current_goal = goal
        server._behavior_started = True
        server._active_behavior_id = 51
        server._current_state = 9
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.FAILED, behavior_id=51, args=[]))

        self.assertEqual(1, goal.aborted_count)
        self.assertEqual('failed', server.outcome)
        self.assertFalse(server.running)

    def test_execute_cb_returns_result_after_cleanup(self):
        """Execute callback should return the stored outcome and clear transient state."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()
        server.running = False
        server.outcome = 'success'

        result = server._execute_cb(goal)

        self.assertEqual('success', result.outcome)
        self.assertEqual('', server.outcome)
        self.assertFalse(server.running)

    def test_state_cb_publishes_feedback_for_active_goal(self):
        """State updates should be forwarded as action feedback while the goal is active."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()
        feedback = []
        goal.publish_feedback = lambda msg: feedback.append(msg.current_state)
        server._current_goal = goal

        server._state_cb(SimpleNamespace(data=42))

        self.assertEqual(42, server._current_state)
        self.assertEqual([42], feedback)

    def test_status_cb_ignores_mismatched_behavior_id(self):
        """Ignore terminal statuses that belong to another behavior id."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()
        server._current_goal = goal
        server._behavior_started = True
        server._active_behavior_id = 51
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.FINISHED, behavior_id=99, args=['done']))

        self.assertEqual(0, goal.succeeded_count)
        self.assertTrue(server.running)
        self.assertEqual('', server.outcome)

    def test_status_cb_error_without_active_goal_does_not_crash(self):
        """Ignore stray error status safely when no pending or active behavior is tracked."""
        server = self._make_server(_FakeBehaviorLibrary())
        server._current_goal = None
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.ERROR, behavior_id=0, args=[]))

        self.assertEqual('', server.outcome)
        self.assertTrue(server.running)
        self.assertEqual(1, len(server._node.get_logger().warnings))

    def test_status_cb_finished_without_active_goal_does_not_crash(self):
        """Handle finished status safely even if there is no active goal handle left."""
        server = self._make_server(_FakeBehaviorLibrary())
        server._current_goal = None
        server._behavior_started = True
        server._active_behavior_id = 51
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.FINISHED, behavior_id=51, args=['done']))

        self.assertEqual('success', server.outcome)
        self.assertFalse(server.running)

    def test_status_cb_failed_without_active_goal_does_not_crash(self):
        """Handle failed status safely even if there is no active goal handle left."""
        server = self._make_server(_FakeBehaviorLibrary())
        server._current_goal = None
        server._behavior_started = True
        server._active_behavior_id = 51
        server._current_state = 9
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.FAILED, behavior_id=51, args=[]))

        self.assertEqual('failed', server.outcome)
        self.assertFalse(server.running)

    def test_status_cb_warning_without_active_goal_is_ignored(self):
        """Ignore non-terminal WARNING safely even if there is no active goal handle left."""
        server = self._make_server(_FakeBehaviorLibrary())
        server._current_goal = None
        server._behavior_started = True
        server._active_behavior_id = 51
        server._current_state = 9
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.WARNING, behavior_id=51, args=[]))

        self.assertEqual('', server.outcome)
        self.assertTrue(server.running)

    def test_status_cb_ignores_prestart_warning(self):
        """Ignore WARNING before startup because onboard no longer uses it as terminal status."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()
        server._current_goal = goal
        server._requested_behavior_id = 77
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.WARNING, behavior_id=77, args=[]))

        self.assertEqual(0, goal.aborted_count)
        self.assertEqual('', server.outcome)
        self.assertTrue(server.running)
        self.assertEqual(77, server._requested_behavior_id)

    def test_status_cb_aborts_matching_prestart_error(self):
        """Abort the accepted goal only when ERROR matches the pending requested behavior id."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()
        server._current_goal = goal
        server._requested_behavior_id = 77
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.ERROR, behavior_id=77, args=[]))

        self.assertEqual(1, goal.aborted_count)
        self.assertEqual('error', server.outcome)
        self.assertFalse(server.running)
        self.assertIsNone(server._requested_behavior_id)
        self.assertIsNone(server._active_behavior_id)

    def test_status_cb_aborts_matching_prestart_failed(self):
        """Abort the accepted goal when FAILED matches the pending requested behavior id."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()
        server._current_goal = goal
        server._requested_behavior_id = 77
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.FAILED, behavior_id=77, args=[]))

        self.assertEqual(1, goal.aborted_count)
        self.assertEqual('failed', server.outcome)
        self.assertFalse(server.running)
        self.assertIsNone(server._requested_behavior_id)
        self.assertIsNone(server._active_behavior_id)

    def test_status_cb_ignores_unrelated_prestart_error(self):
        """Ignore ERROR statuses that do not belong to the currently pending behavior request."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()
        server._current_goal = goal
        server._requested_behavior_id = 77
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.ERROR, behavior_id=88, args=[]))

        self.assertEqual(0, goal.aborted_count)
        self.assertEqual('', server.outcome)
        self.assertTrue(server.running)
        self.assertEqual(77, server._requested_behavior_id)
        self.assertEqual(1, len(server._node.get_logger().warnings))

    def test_status_cb_ignores_unrelated_prestart_failed(self):
        """Ignore FAILED statuses that do not belong to the currently pending behavior request."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()
        server._current_goal = goal
        server._requested_behavior_id = 77
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.FAILED, behavior_id=88, args=[]))

        self.assertEqual(0, goal.aborted_count)
        self.assertEqual('', server.outcome)
        self.assertTrue(server.running)
        self.assertEqual(77, server._requested_behavior_id)
        self.assertEqual(1, len(server._node.get_logger().warnings))

    def test_status_cb_ignores_unrelated_running_error(self):
        """Do not abort the active goal when ERROR belongs to another behavior id."""
        server = self._make_server(_FakeBehaviorLibrary())
        goal = _FakeGoal()
        server._current_goal = goal
        server._behavior_started = True
        server._active_behavior_id = 51
        server.running = True

        server._status_cb(SimpleNamespace(code=BEStatus.ERROR, behavior_id=99, args=[]))

        self.assertEqual(0, goal.aborted_count)
        self.assertEqual('', server.outcome)
        self.assertTrue(server.running)


if __name__ == '__main__':
    unittest.main()
