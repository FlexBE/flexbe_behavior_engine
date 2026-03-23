#!/usr/bin/env python3

"""Unit tests for ComplexActionServer goal acceptance behavior."""

import queue
import threading
import unittest
from unittest.mock import patch

from flexbe_input.complex_action_server import ComplexActionServer

from rclpy.action import CancelResponse, GoalResponse


class _FakeGoalHandle:
    """Simple stand-in for an action server goal handle."""

    def __init__(self):
        self.succeed_calls = 0

    def succeed(self, response=None):
        self.succeed_calls += 1


class TestComplexActionServer(unittest.TestCase):
    """Validate local ComplexActionServer state transitions."""

    def test_init_creates_action_server_and_starts_execute_thread(self):
        """Construction with an execute callback should start the worker thread and create the ROS action server."""
        started = []

        class _Thread:

            def __init__(self, _group, target):
                self.target = target

            def start(self):
                started.append(self.target)

            def join(self):
                return None

        with patch('flexbe_input.complex_action_server.threading.Thread', _Thread), \
                patch('flexbe_input.complex_action_server.ActionServer', return_value='action-server') as action_server:
            server = ComplexActionServer(object(), 'test_action', object(), execute_cb=lambda goal, handle: None)

        self.assertEqual(server.action_server, 'action-server')
        self.assertEqual(started, [server.executeLoop])
        action_server.assert_called_once()

    def test_del_terminates_and_joins_execute_thread(self):
        """Destruction should request termination and wait for the execute thread."""
        joined = []
        server = ComplexActionServer.__new__(ComplexActionServer)
        server.execute_callback = object()
        server.terminate_mutex = threading.RLock()
        server.need_to_terminate = False
        server.execute_thread = type('_Thread', (), {'join': staticmethod(lambda: joined.append(True))})()

        server.__del__()

        self.assertTrue(server.need_to_terminate)
        self.assertEqual(joined, [True])

    def test_is_active_is_false_without_current_goal(self):
        """Server should report inactive when no goal has been accepted yet."""
        server = ComplexActionServer.__new__(ComplexActionServer)
        server.current_goal = None

        self.assertFalse(server.is_active())

    def test_is_active_requires_goal_object_and_active_flag(self):
        """Activity checks should reject empty goal handles and accept live ones."""
        server = ComplexActionServer.__new__(ComplexActionServer)
        server.current_goal = type('_GoalHandle', (), {'get_goal': staticmethod(lambda: None), 'is_active': True})()
        self.assertFalse(server.is_active())

        server.current_goal = type('_GoalHandle', (), {'get_goal': staticmethod(lambda: object()), 'is_active': True})()
        self.assertTrue(server.is_active())

    def test_accept_new_goal_does_not_complete_goal_early(self):
        """Accepting a queued goal should not mark it succeeded before execution."""
        server = ComplexActionServer.__new__(ComplexActionServer)
        server.goals_received_ = 1
        server.goal_queue_ = queue.Queue()
        server.current_goal = None

        goal_handle = _FakeGoalHandle()
        server.goal_queue_.put(goal_handle)

        with patch('flexbe_input.complex_action_server.Logger.logdebug'):
            returned = server.accept_new_goal()

        self.assertIs(goal_handle, returned)
        self.assertIs(goal_handle, server.current_goal)
        self.assertEqual(0, goal_handle.succeed_calls)
        self.assertEqual(0, server.goals_received_)

    def test_register_goal_callback_ignores_manual_registration_when_execute_callback_exists(self):
        """Manual goal callbacks should be rejected when the execute thread owns dispatch."""
        server = ComplexActionServer.__new__(ComplexActionServer)
        server.execute_callback = object()
        server.goal_callback = None
        server.terminate_mutex = threading.RLock()
        server.need_to_terminate = False
        server.execute_thread = type('_Thread', (), {'join': staticmethod(lambda: None)})()

        with patch('flexbe_input.complex_action_server.Logger.logwarn') as log_warn:
            server.register_goal_callback(lambda goal: goal)

        self.assertIsNone(server.goal_callback)
        log_warn.assert_called_once()

    def test_internal_goal_callback_queues_goal_and_notifies_waiters(self):
        """New goals should be accepted, queued, and counted for later execution."""
        server = ComplexActionServer.__new__(ComplexActionServer)
        server.goal_queue_ = queue.Queue()
        server.goals_received_ = 0
        server.new_goal = False
        server.next_goal = None
        server.execute_condition = type(
            '_Condition',
            (),
            {
                'acquire': staticmethod(lambda: None),
                'notify': staticmethod(lambda: notifications.append('notify')),
                'release': staticmethod(lambda: notifications.append('release')),
            },
        )()
        notifications = []

        goal = type('_Goal', (), {'goal_id': 'goal-1'})()

        with patch('flexbe_input.complex_action_server.Logger.localinfo'):
            response = server.internal_goal_callback(goal)

        self.assertEqual(1, server.goals_received_)
        self.assertTrue(server.new_goal)
        self.assertIs(server.next_goal, goal)
        self.assertIs(server.goal_queue_.get_nowait(), goal)
        self.assertEqual(['notify', 'release'], notifications)
        self.assertEqual(GoalResponse.ACCEPT, response)

    def test_set_succeeded_and_aborted_use_default_result_when_none_is_provided(self):
        """Completion helpers should fall back to the action type when no explicit result is passed."""
        server = ComplexActionServer.__new__(ComplexActionServer)
        server.action_server = type('_ActionServer', (), {'action_type': object()})()
        goal_handle = type(
            '_GoalHandle',
            (),
            {
                '__init__': lambda self: setattr(self, 'calls', []),
                'succeed': lambda self: self.calls.append('succeed'),
                'abort': lambda self: self.calls.append('abort'),
            },
        )()

        succeeded_result = server.set_succeeded(goal_handle=goal_handle)
        aborted_result = server.set_aborted(goal_handle=goal_handle)

        self.assertIs(server.action_server.action_type, succeeded_result)
        self.assertIs(server.action_server.action_type, aborted_result)
        self.assertEqual(['succeed', 'abort'], goal_handle.calls)

    def test_publish_feedback_and_run_goal_delegate_to_goal_and_callback(self):
        """Feedback publishing and goal execution should pass through to the configured targets."""
        feedback_messages = []
        executed = []
        server = ComplexActionServer.__new__(ComplexActionServer)
        server.current_goal = type('_GoalHandle', (), {'publish_feedback': staticmethod(feedback_messages.append)})()
        server.execute_callback = lambda goal, handle: executed.append((goal, handle))
        server.terminate_mutex = threading.RLock()
        server.need_to_terminate = False
        server.execute_thread = type('_Thread', (), {'join': staticmethod(lambda: None)})()

        goal = object()
        handle = object()
        feedback = object()

        server.publish_feedback(feedback)
        server.run_goal(goal, handle)

        self.assertEqual(feedback_messages, [feedback])
        self.assertEqual(executed, [(goal, handle)])

    def test_register_goal_callback_stores_callback_without_execute_thread(self):
        """Manual goal callbacks should be stored when no execute callback is configured."""
        server = ComplexActionServer.__new__(ComplexActionServer)
        server.execute_callback = None
        server.goal_callback = None

        def callback(goal):
            return goal

        server.register_goal_callback(callback)

        self.assertIs(callback, server.goal_callback)

    def test_internal_goal_callback_rejects_on_queue_failure(self):
        """Queueing failures should reject the goal and release the waiting condition."""
        notifications = []
        server = ComplexActionServer.__new__(ComplexActionServer)
        server.goals_received_ = 0
        server.new_goal = False
        server.next_goal = None
        server.goal_queue_ = type('_Queue', (), {'put': staticmethod(lambda goal: (_ for _ in ()).throw(RuntimeError('boom')))})()
        server.execute_condition = type(
            '_Condition',
            (),
            {
                'acquire': staticmethod(lambda: notifications.append('acquire')),
                'notify': staticmethod(lambda: notifications.append('notify')),
                'release': staticmethod(lambda: notifications.append('release')),
            },
        )()

        goal = type('_Goal', (), {'goal_id': 'goal-1'})()

        with patch('flexbe_input.complex_action_server.Logger.localinfo'), \
                patch('flexbe_input.complex_action_server.Logger.logerr') as logerr:
            response = server.internal_goal_callback(goal)

        self.assertEqual(response, GoalResponse.REJECT)
        self.assertEqual(notifications, ['acquire', 'release'])
        logerr.assert_called_once()

    def test_internal_preempt_callback_accepts_cancel_requests(self):
        """Preempt requests should always be accepted by the wrapper callback."""
        server = ComplexActionServer.__new__(ComplexActionServer)

        self.assertEqual(server.internal_preempt_callback(object()), CancelResponse.ACCEPT)

    def test_execute_loop_logs_and_returns_when_goal_arrives_without_execute_callback(self):
        """The execute loop should fail fast if it accepts a goal but no execute callback is configured."""
        server = ComplexActionServer.__new__(ComplexActionServer)
        server.terminate_mutex = threading.RLock()
        server.need_to_terminate = False
        server.execute_callback = None
        server.execute_thread = None
        server.execute_condition = type(
            '_Condition',
            (),
            {
                '__enter__': staticmethod(lambda: None),
                '__exit__': staticmethod(lambda exc_type, exc, tb: False),
                'wait': staticmethod(lambda timeout: None),
            },
        )()
        accepted = object()
        server.is_new_goal_available = lambda: True
        server.accept_new_goal = lambda: accepted

        with patch('flexbe_input.complex_action_server.rclpy.ok', side_effect=[True]), \
                patch('flexbe_input.complex_action_server.Logger.logdebug'), \
                patch('flexbe_input.complex_action_server.Logger.logerr') as logerr:
            result = server.executeLoop()

        self.assertIsNone(result)
        logerr.assert_called_once()

    def test_execute_loop_starts_worker_thread_for_new_goal(self):
        """Available goals should be dispatched into a worker thread with the goal payload and handle."""
        started = []

        class _Thread:

            def __init__(self, target, args):
                self.target = target
                self.args = args

            def start(self):
                started.append((self.target, self.args))

        class _Condition:

            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, tb):
                return False

            def wait(self, timeout):
                return None

        goal_payload = object()
        goal_handle = type('_GoalHandle', (), {'get_goal': staticmethod(lambda: goal_payload)})()
        server = ComplexActionServer.__new__(ComplexActionServer)
        server.terminate_mutex = threading.RLock()
        server.need_to_terminate = False
        server.execute_callback = lambda goal, handle: None
        server.execute_thread = type('_ThreadHandle', (), {'join': staticmethod(lambda: None)})()
        server.execute_condition = _Condition()
        server.is_new_goal_available = lambda: True
        server.accept_new_goal = lambda: goal_handle
        server.run_goal = lambda goal, handle: None

        with patch('flexbe_input.complex_action_server.rclpy.ok', side_effect=[True, False]), \
                patch('flexbe_input.complex_action_server.threading.Thread', _Thread), \
                patch('flexbe_input.complex_action_server.Logger.logdebug'):
            server.executeLoop()

        self.assertEqual(started, [(server.run_goal, (goal_payload, goal_handle))])

    def test_execute_loop_aborts_when_thread_creation_raises(self):
        """Thread creation failures should abort the goal and log the exception."""
        class _Condition:

            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, tb):
                return False

            def wait(self, timeout):
                return None

        goal_handle = type('_GoalHandle', (), {'get_goal': staticmethod(lambda: object())})()
        server = ComplexActionServer.__new__(ComplexActionServer)
        server.terminate_mutex = threading.RLock()
        server.need_to_terminate = False
        server.execute_callback = lambda goal, handle: None
        server.execute_thread = type('_ThreadHandle', (), {'join': staticmethod(lambda: None)})()
        server.execute_condition = _Condition()
        server.is_new_goal_available = lambda: True
        server.accept_new_goal = lambda: goal_handle
        aborted = []
        server.set_aborted = lambda result=None, text='', goal_handle=None: aborted.append((result, text, goal_handle))

        def _raise_thread(*args, **kwargs):
            raise RuntimeError('boom')

        with patch('flexbe_input.complex_action_server.rclpy.ok', side_effect=[True, False]), \
                patch('flexbe_input.complex_action_server.threading.Thread', side_effect=_raise_thread), \
                patch('flexbe_input.complex_action_server.Logger.logdebug'), \
                patch('flexbe_input.complex_action_server.Logger.logerr') as logerr:
            server.executeLoop()

        self.assertEqual(len(aborted), 1)
        self.assertIn('Exception in execute callback: boom', aborted[0][1])
        logerr.assert_called_once()


if __name__ == '__main__':
    unittest.main()
