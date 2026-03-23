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

"""Unit tests for ProxyActionClient cancel-path validation."""

import unittest
from unittest.mock import patch

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal

from flexbe_core.core.exceptions import ProxyAvailabilityError, ProxyTypeError, ShutdownError
from flexbe_core.proxy.proxy_action_client import ProxyActionClient


class _FakeFuture:

    def __init__(self, result_obj=None, error=None, done=True):
        self._result_obj = result_obj
        self._error = error
        self._done = done
        self._callbacks = []

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result_obj

    def done(self):
        return self._done

    def resolve(self, result_obj=None, error=None):
        if result_obj is not None:
            self._result_obj = result_obj
        self._error = error
        self._done = True
        callbacks = list(self._callbacks)
        for callback in callbacks:
            callback(self)


class _FakeCancelFuture:

    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, cb):
        self.callbacks.append(cb)


class _FakeGoalHandle:

    def __init__(self, cancel_future=None):
        self._cancel_future = cancel_future

    def cancel_goal_async(self):
        return self._cancel_future


class _FakeResultResponse:

    def __init__(self, result=None, status=GoalStatus.STATUS_SUCCEEDED):
        self.result = result
        self.status = status


class _FakeFeedback:

    def __init__(self, feedback):
        self.feedback = feedback


class _FakeExecutor:

    def __init__(self):
        self.tasks = []

    def create_task(self, func, *args):
        self.tasks.append((func, args))


class _FakeNode:

    def __init__(self):
        self.executor = _FakeExecutor()


class _FakeDestroyNode(_FakeNode):

    def __init__(self, destroy_result=True, destroy_error=None, time_ns_sequence=None):
        super().__init__()
        self._destroy_result = destroy_result
        self._destroy_error = destroy_error
        self._time_ns_sequence = list(time_ns_sequence or [0])
        self.destroy_calls = []

    def destroy_client(self, client):
        self.destroy_calls.append(client)
        if self._destroy_error is not None:
            raise self._destroy_error
        return self._destroy_result

    def get_clock(self):
        return self

    def now(self):
        if len(self._time_ns_sequence) > 1:
            value = self._time_ns_sequence.pop(0)
        else:
            value = self._time_ns_sequence[0]

        class _FakeDelta:

            def __init__(self, nanoseconds):
                self.nanoseconds = nanoseconds

            def __lt__(self, other):
                return self.nanoseconds < other.nanoseconds

        class _FakeTime:

            def __init__(self, nanoseconds):
                self.nanoseconds = nanoseconds

            def __sub__(self, other):
                return _FakeDelta(self.nanoseconds - other.nanoseconds)

        return _FakeTime(value)


class _FakeActionClientEntry:

    def __init__(self, action_type, ready=True):
        self._action_type = action_type
        self._ready = ready
        self.wait_calls = []

    def server_is_ready(self):
        return self._ready

    def wait_for_server(self, wait_duration):
        self.wait_calls.append(wait_duration)
        return self._ready


def _make_action_type(name):
    """Create a minimal action type with a stable Goal class."""

    class _Goal:
        """Placeholder goal type."""

    return type(name, (), {'Goal': _Goal})


class TestProxyActionClientCancel(unittest.TestCase):
    """Validate cancel and shutdown guard paths in proxy action client."""

    def setUp(self):
        """Reset proxy static state before each test."""
        ProxyActionClient._current_goal.clear()
        ProxyActionClient._result_status.clear()
        ProxyActionClient._clients.clear()
        ProxyActionClient._result.clear()
        ProxyActionClient._feedback.clear()
        ProxyActionClient._has_active_goal.clear()
        ProxyActionClient._node = None
        ProxyActionClient._is_shutting_down = False
        ProxyActionClient._client_generation_counter = 0

    def test_resolve_goal_handle_missing_future(self):
        """Resolve should fail when no active goal future exists."""
        with self.assertRaises(ProxyAvailabilityError):
            ProxyActionClient._resolve_goal_handle_for_cancel('topic')

    def test_resolve_goal_handle_missing_handle(self):
        """Resolve should fail when future resolves to no goal handle."""
        ProxyActionClient._current_goal['topic'] = _FakeFuture(result_obj=None)
        with self.assertRaises(ProxyAvailabilityError):
            ProxyActionClient._resolve_goal_handle_for_cancel('topic')

    def test_resolve_goal_handle_invalid_type(self):
        """Resolve should fail when goal handle type is invalid."""
        ProxyActionClient._current_goal['topic'] = _FakeFuture(result_obj=object())
        with self.assertRaises(ProxyTypeError):
            ProxyActionClient._resolve_goal_handle_for_cancel('topic')

    def test_cancel_sets_canceling_and_registers_callback(self):
        """Cancel should mark status and register completion callback."""
        cancel_future = _FakeCancelFuture()
        goal_handle = _FakeGoalHandle(cancel_future=cancel_future)
        current_goal_future = _FakeFuture(result_obj=goal_handle)
        ProxyActionClient._clients['topic'] = {'client': object(), 'generation': 1, 'count': 1}
        ProxyActionClient._current_goal['topic'] = current_goal_future

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localinfo'):
            ProxyActionClient.cancel('topic')

        self.assertEqual(ProxyActionClient._result_status['topic'], GoalStatus.STATUS_CANCELING)
        self.assertEqual(len(cancel_future.callbacks), 1)
        self.assertIs(ProxyActionClient._current_goal['topic'], current_goal_future)

    def test_cancel_handles_result_exception(self):
        """Cancel should preserve active-goal bookkeeping if cancel setup fails."""
        ProxyActionClient._has_active_goal['topic'] = True
        current_goal_future = _FakeFuture(error=RuntimeError('boom'))
        ProxyActionClient._current_goal['topic'] = current_goal_future
        ProxyActionClient._result_status['topic'] = GoalStatus.STATUS_ACCEPTED

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localinfo'):
            ProxyActionClient.cancel('topic')

        self.assertIs(ProxyActionClient._current_goal['topic'], current_goal_future)
        self.assertTrue(ProxyActionClient._has_active_goal['topic'])
        self.assertEqual(ProxyActionClient._result_status['topic'], GoalStatus.STATUS_ACCEPTED)

    def test_cancel_handles_missing_goal_handle_without_dropping_active_goal(self):
        """Cancel should not drop active-goal tracking when cancel setup cannot resolve the handle."""
        ProxyActionClient._clients['topic'] = {'client': object(), 'generation': 1, 'count': 1}
        ProxyActionClient._has_active_goal['topic'] = True
        current_goal_future = _FakeFuture(result_obj=None)
        ProxyActionClient._current_goal['topic'] = current_goal_future
        ProxyActionClient._result_status['topic'] = GoalStatus.STATUS_ACCEPTED

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localinfo'):
            ProxyActionClient.cancel('topic')

        self.assertIs(ProxyActionClient._current_goal['topic'], current_goal_future)
        self.assertTrue(ProxyActionClient._has_active_goal['topic'])
        self.assertEqual(ProxyActionClient._result_status['topic'], GoalStatus.STATUS_ACCEPTED)

    def test_cancel_waits_for_pending_goal_future_then_sends_cancel(self):
        """Cancel should queue against a pending send_goal future."""
        cancel_future = _FakeCancelFuture()
        goal_handle = _FakeGoalHandle(cancel_future=cancel_future)
        pending_future = _FakeFuture(done=False)
        ProxyActionClient._clients['topic'] = {'client': object(), 'generation': 1, 'count': 1}
        ProxyActionClient._current_goal['topic'] = pending_future
        ProxyActionClient._has_active_goal['topic'] = True

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localinfo'):
            ProxyActionClient.cancel('topic')

        self.assertEqual(ProxyActionClient._result_status['topic'], GoalStatus.STATUS_CANCELING)
        self.assertEqual(len(cancel_future.callbacks), 0)

        pending_future.resolve(result_obj=goal_handle)

        self.assertEqual(len(cancel_future.callbacks), 1)
        self.assertIs(ProxyActionClient._current_goal['topic'], pending_future)

    def test_shutdown_handles_invalid_client_entry(self):
        """Shutdown should log and continue on invalid client dictionary entries."""
        ProxyActionClient._clients['topic'] = 'bad-entry'

        with patch('flexbe_core.proxy.proxy_action_client.Logger.error') as log_error, \
                patch('builtins.print'):
            ProxyActionClient.shutdown()

        self.assertEqual(log_error.call_count, 1)
        self.assertFalse(ProxyActionClient._clients)

    def test_shutdown_handles_missing_node_for_live_client(self):
        """Shutdown should log and continue when node is missing for live clients."""
        ProxyActionClient._clients['topic'] = {'client': object(), 'count': 1}
        ProxyActionClient._node = None

        with patch('flexbe_core.proxy.proxy_action_client.Logger.error') as log_error, \
                patch('builtins.print'):
            ProxyActionClient.shutdown()

        self.assertEqual(log_error.call_count, 1)
        self.assertFalse(ProxyActionClient._clients)

    def test_done_callback_ignores_late_callback_after_shutdown(self):
        """Late goal completion callbacks should not repopulate cleared state after shutdown."""
        ProxyActionClient._is_shutting_down = True

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localinfo'):
            ProxyActionClient._done_callback(_FakeFuture(result_obj=_FakeGoalHandle()), 'topic', 1)

        self.assertFalse(ProxyActionClient._current_goal)
        self.assertFalse(ProxyActionClient._result_status)
        self.assertFalse(ProxyActionClient._has_active_goal)

    def test_result_callback_ignores_late_callback_after_shutdown(self):
        """Late result callbacks should not repopulate cleared state after shutdown."""
        ProxyActionClient._is_shutting_down = True

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localwarn'):
            ProxyActionClient._result_callback(_FakeFuture(result_obj=_FakeResultResponse(result='ok')), 'topic', 1)

        self.assertFalse(ProxyActionClient._result)
        self.assertFalse(ProxyActionClient._result_status)
        self.assertFalse(ProxyActionClient._has_active_goal)
        self.assertFalse(ProxyActionClient._current_goal)

    def test_feedback_callback_ignores_late_callback_after_shutdown(self):
        """Late feedback callbacks should not repopulate cleared state after shutdown."""
        ProxyActionClient._is_shutting_down = True

        ProxyActionClient._feedback_callback('topic', _FakeFeedback('late-feedback'), 1)

        self.assertFalse(ProxyActionClient._feedback)

    def test_remove_client_clears_all_topic_state_on_last_reference(self):
        """Removing the last client reference should clear all per-topic action state."""
        topic = 'topic'
        client = object()
        ProxyActionClient._node = _FakeNode()
        ProxyActionClient._clients[topic] = {'client': client, 'generation': 1, 'count': 1}
        ProxyActionClient._result[topic] = 'result'
        ProxyActionClient._feedback[topic] = 'feedback'
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_SUCCEEDED
        ProxyActionClient._has_active_goal[topic] = True
        ProxyActionClient._current_goal[topic] = _FakeFuture(result_obj=None)

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localdebug'):
            ProxyActionClient.remove_client(topic)

        self.assertNotIn(topic, ProxyActionClient._clients)
        self.assertNotIn(topic, ProxyActionClient._result)
        self.assertNotIn(topic, ProxyActionClient._feedback)
        self.assertNotIn(topic, ProxyActionClient._result_status)
        self.assertNotIn(topic, ProxyActionClient._has_active_goal)
        self.assertNotIn(topic, ProxyActionClient._current_goal)
        self.assertEqual(len(ProxyActionClient._node.executor.tasks), 1)
        destroy_func, destroy_args = ProxyActionClient._node.executor.tasks[0]
        self.assertIs(destroy_func.__func__, ProxyActionClient.destroy_client.__func__)
        self.assertEqual(destroy_args, (client, topic))

    def test_result_callback_ignores_stale_client_generation(self):
        """Late results from a replaced client should not update the new client state."""
        topic = 'topic'
        ProxyActionClient._clients[topic] = {'client': object(), 'generation': 2, 'count': 1}
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_UNKNOWN
        ProxyActionClient._has_active_goal[topic] = True
        ProxyActionClient._current_goal[topic] = _FakeFuture()

        ProxyActionClient._result_callback(_FakeFuture(result_obj=_FakeResultResponse(result='stale')), topic, 1)

        self.assertNotIn(topic, ProxyActionClient._result)
        self.assertEqual(ProxyActionClient._result_status[topic], GoalStatus.STATUS_UNKNOWN)
        self.assertTrue(ProxyActionClient._has_active_goal[topic])
        self.assertIsNotNone(ProxyActionClient._current_goal[topic])

    def test_feedback_callback_ignores_stale_client_generation(self):
        """Late feedback from a replaced client should not update the new client state."""
        topic = 'topic'
        ProxyActionClient._clients[topic] = {'client': object(), 'generation': 2, 'count': 1}

        ProxyActionClient._feedback_callback(topic, _FakeFeedback('stale-feedback'), 1)

        self.assertNotIn(topic, ProxyActionClient._feedback)

    def test_cancel_callback_ignores_stale_client_generation(self):
        """Late cancel completion from a replaced client should not update the new client state."""
        topic = 'topic'
        ProxyActionClient._clients[topic] = {'client': object(), 'generation': 2, 'count': 1}
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_CANCELING
        ProxyActionClient._has_active_goal[topic] = True
        ProxyActionClient._current_goal[topic] = _FakeFuture()

        ProxyActionClient._cancel_callback(
            _FakeFuture(result_obj=type('CancelResult', (), {
                'return_code': CancelGoal.Response.ERROR_NONE})()),
            topic, 1
        )

        self.assertEqual(ProxyActionClient._result_status[topic], GoalStatus.STATUS_CANCELING)
        self.assertTrue(ProxyActionClient._has_active_goal[topic])
        self.assertIsNotNone(ProxyActionClient._current_goal[topic])

    def test_setup_client_recreate_clears_stale_topic_state(self):
        """Recreating a same-name action client should clear prior per-topic state."""
        topic = 'topic'
        old_action_type = _make_action_type('SharedAction')
        new_action_type = _make_action_type('SharedAction')
        old_client = type('OldClient', (), {'_action_type': old_action_type})()
        new_client = type('NewClient', (), {'_action_type': new_action_type})()
        ProxyActionClient._node = _FakeNode()
        ProxyActionClient._clients[topic] = {'client': old_client, 'generation': 1, 'count': 1}
        ProxyActionClient._result[topic] = 'stale-result'
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_SUCCEEDED
        ProxyActionClient._feedback[topic] = 'stale-feedback'
        ProxyActionClient._has_active_goal[topic] = True
        ProxyActionClient._current_goal[topic] = _FakeFuture()

        with patch('flexbe_core.proxy.proxy_action_client.ActionClient', return_value=new_client), \
                patch('flexbe_core.proxy.proxy_action_client.Logger.localinfo'), \
                patch.object(ProxyActionClient, '_check_topic_available', return_value=True):
            ProxyActionClient.setup_client(topic, new_action_type, wait_duration=0.01)

        self.assertNotIn(topic, ProxyActionClient._result)
        self.assertNotIn(topic, ProxyActionClient._result_status)
        self.assertNotIn(topic, ProxyActionClient._feedback)
        self.assertNotIn(topic, ProxyActionClient._has_active_goal)
        self.assertNotIn(topic, ProxyActionClient._current_goal)
        self.assertIs(ProxyActionClient._clients[topic]['client'], new_client)

    def test_shutdown_topic_client_handles_none_and_destroy_failure(self):
        """Shutdown helper should accept None entries and wrap destroy runtime errors."""
        ProxyActionClient._clients['empty'] = {'client': object(), 'count': 1}
        ProxyActionClient._node = _FakeDestroyNode(destroy_error=RuntimeError('boom'))

        ProxyActionClient._shutdown_topic_client('empty_none', None)
        self.assertIsNone(ProxyActionClient._clients['empty_none'])

        with self.assertRaisesRegex(ShutdownError, 'Failed to destroy client'):
            ProxyActionClient._shutdown_topic_client('empty', {'client': object()})

    def test_setup_client_deprecated_and_reference_paths(self):
        """Deprecated wrapper should delegate, and same-type setup should retain references."""
        topic = 'topic'
        action_type = _make_action_type('DemoAction')
        ProxyActionClient._node = _FakeNode()

        with patch.object(ProxyActionClient, 'setup_client') as setup_client, \
                patch('flexbe_core.proxy.proxy_action_client.Logger.localerr') as localerr:
            ProxyActionClient.setupClient(topic, action_type, wait_duration=1.5)

        localerr.assert_called_once()
        setup_client.assert_called_once_with(topic, action_type, 1.5)

        ready_client = _FakeActionClientEntry(action_type)
        ProxyActionClient._clients[topic] = {'client': ready_client, 'generation': 1, 'count': 1}
        ProxyActionClient.setup_client(topic, action_type, wait_duration=None)
        self.assertEqual(ProxyActionClient._clients[topic]['count'], 2)

        with self.assertRaises(ProxyTypeError):
            ProxyActionClient.setup_client(topic, _make_action_type('OtherAction'), wait_duration=None)

    def test_is_available_and_state_helpers_cover_uninitialized_topics(self):
        """Availability and result/feedback helpers should handle empty state cleanly."""
        topic = 'topic'
        client = _FakeActionClientEntry(_make_action_type('DemoAction'), ready=False)
        ProxyActionClient._clients[topic] = {'client': client, 'generation': 1, 'count': 1}
        ProxyActionClient._result[topic] = 'result'
        ProxyActionClient._feedback[topic] = 'feedback'
        ProxyActionClient._result_status[topic] = 99
        ProxyActionClient._has_active_goal[topic] = True

        with patch('flexbe_core.proxy.proxy_action_client.Logger.logerr') as logerr, \
                patch('flexbe_core.proxy.proxy_action_client.Logger.localwarn') as localwarn:
            self.assertFalse(ProxyActionClient.is_available('missing'))
            self.assertEqual(logerr.call_count, 1)
            ProxyActionClient._clients['empty'] = {'client': None, 'generation': 1, 'count': 1}
            self.assertFalse(ProxyActionClient.is_available('empty'))
            self.assertEqual(logerr.call_count, 2)
            self.assertFalse(ProxyActionClient.is_available(topic))

            self.assertTrue(ProxyActionClient.has_result(topic))
            self.assertEqual(ProxyActionClient.get_result(topic, clear=True), 'result')
            self.assertFalse(ProxyActionClient.has_result(topic))
            self.assertTrue(ProxyActionClient.has_feedback(topic))
            self.assertEqual(ProxyActionClient.get_feedback(topic, clear=True), 'feedback')
            self.assertFalse(ProxyActionClient.has_feedback(topic))
            self.assertEqual(ProxyActionClient.get_status_string(topic), 'Unknown Status')
            self.assertTrue(ProxyActionClient.is_active(topic))

            ProxyActionClient.remove_result(topic)
            localwarn.assert_called_once()
            self.assertEqual(99, ProxyActionClient._result_status[topic])
            self.assertTrue(ProxyActionClient._has_active_goal[topic])

            ProxyActionClient.remove_feedback(topic)
            self.assertIsNone(ProxyActionClient._feedback[topic])

    def test_verify_action_status_and_check_topic_available_paths(self):
        """Status verification and availability checks should cover terminal and timeout cases."""
        topic = 'topic'
        action_type = _make_action_type('DemoAction')
        ProxyActionClient._node = _FakeDestroyNode(time_ns_sequence=[0, 0, 2_000_000, 10_000_000, 15_000_000, 25_000_000])
        ProxyActionClient._clients[topic] = {
            'client': _FakeActionClientEntry(action_type, ready=True),
            'generation': 1,
            'count': 1,
        }
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_SUCCEEDED

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localinfo') as localinfo:
            self.assertEqual(ProxyActionClient.verify_action_status('missing'), None)
            self.assertEqual(ProxyActionClient.verify_action_status(topic, wait_duration=0.01),
                             (True, GoalStatus.STATUS_SUCCEEDED))
            self.assertGreaterEqual(localinfo.call_count, 1)

        ProxyActionClient._node = _FakeDestroyNode(time_ns_sequence=[0, 0, 30_000_000, 60_000_000])
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_ACCEPTED
        self.assertEqual(ProxyActionClient.verify_action_status(topic, wait_duration=0.01),
                         (False, GoalStatus.STATUS_ACCEPTED))

        class _ImmediateTimer:

            def __init__(self, _delay, callback, args):
                self._callback = callback
                self._args = args

            def start(self):
                self._callback(*self._args)

            def cancel(self):
                return None

        with patch('flexbe_core.proxy.proxy_action_client.Timer', _ImmediateTimer), \
                patch('flexbe_core.proxy.proxy_action_client.Logger.loginfo') as loginfo, \
                patch('flexbe_core.proxy.proxy_action_client.Logger.logwarn'):
            self.assertTrue(ProxyActionClient._check_topic_available(topic, wait_duration=3.0))
            loginfo.assert_called_once()

        ProxyActionClient._clients[topic]['client'] = _FakeActionClientEntry(action_type, ready=False)
        with patch('flexbe_core.proxy.proxy_action_client.Logger.logerr') as logerr:
            self.assertFalse(ProxyActionClient._check_topic_available('missing', wait_duration=0.1))
            self.assertFalse(ProxyActionClient._check_topic_available(topic, wait_duration=0.1))
            self.assertEqual(logerr.call_count, 2)

    def test_destroy_client_handles_success_warning_and_exceptions(self):
        """Destroy helper should log success, warning, missing-node, and exception paths."""
        topic = 'topic'

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localinfo') as localinfo, \
                patch('flexbe_core.proxy.proxy_action_client.Logger.localwarn') as localwarn, \
                patch('flexbe_core.proxy.proxy_action_client.Logger.error') as error:
            ProxyActionClient.destroy_client(None, topic)
            localinfo.assert_not_called()

            ProxyActionClient._node = None
            ProxyActionClient.destroy_client(object(), topic)
            localwarn.assert_called_once()

            ProxyActionClient._node = _FakeDestroyNode(destroy_result=True)
            ProxyActionClient.destroy_client(object(), topic)
            self.assertGreaterEqual(localinfo.call_count, 1)

            ProxyActionClient._node = _FakeDestroyNode(destroy_result=False)
            ProxyActionClient.destroy_client(object(), topic)
            self.assertGreaterEqual(localwarn.call_count, 2)

            ProxyActionClient._node = _FakeDestroyNode(destroy_error=RuntimeError('boom'))
            ProxyActionClient.destroy_client(object(), topic)
            error.assert_called_once()


if __name__ == '__main__':
    unittest.main()
