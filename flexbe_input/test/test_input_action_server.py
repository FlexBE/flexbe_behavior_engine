#!/usr/bin/env python3

"""Unit tests for input action server request serialization."""

import threading
import types
import unittest
from unittest.mock import patch

from flexbe_input.input_action_server import InputActionServer, InputActionWorker

from flexbe_msgs.action import BehaviorInput

import rclpy


class TestInputActionServer(unittest.TestCase):
    """Validate non-GUI input action server guard behavior."""

    @staticmethod
    def _noop(*args, **kwargs):
        return None

    def test_input_action_worker_quits_gui_on_external_shutdown(self):
        """Worker shutdown paths should still request the GUI event loop to quit."""
        worker = InputActionWorker(object())

        with patch('flexbe_input.input_action_server.MultiThreadedExecutor', return_value=object()), \
                patch('flexbe_input.input_action_server.rclpy.spin', side_effect=KeyboardInterrupt), \
                patch('flexbe_input.input_action_server.QCoreApplication.quit') as quit_app:
            worker.run()

        quit_app.assert_called_once_with()

    def test_input_action_server_init_wires_server_dialog_and_locks(self):
        """Construction should create the ROS action server, dialog, and request bookkeeping."""
        created_servers = []

        def _fake_action_server(*args, **kwargs):
            created_servers.append((args, kwargs))
            return 'action-server'

        with patch('flexbe_input.input_action_server.Node.__init__', return_value=None) as node_init, \
                patch('flexbe_input.input_action_server.ActionServer', side_effect=_fake_action_server), \
                patch('flexbe_input.input_action_server.InputGUI', return_value='dialog') as input_gui, \
                patch('flexbe_input.input_action_server.Logger.initialize') as logger_init:
            server = InputActionServer()

        node_init.assert_called_once_with('input_action_server')
        self.assertEqual(server._server, 'action-server')
        self.assertEqual(server._input_dialog, 'dialog')
        self.assertEqual(server._action_topic, 'flexbe/behavior_input')
        self.assertIsNone(server._active_goal_handle)
        self.assertIsNone(server._pending_cancel_goal_handle)
        self.assertFalse(server._canceled)
        self.assertIsNone(server._worker)
        input_gui.assert_called_once_with('default')
        logger_init.assert_called_once_with(server)
        self.assertEqual(created_servers[0][1]['execute_callback'], server.execute_callback)
        self.assertEqual(created_servers[0][1]['cancel_callback'], server.cancel_callback)

    def test_execute_callback_rejects_overlapping_request(self):
        """Concurrent requests should be aborted instead of sharing one dialog state."""
        server = InputActionServer.__new__(InputActionServer)
        server._request_lock = threading.Lock()
        server._request_lock.acquire()

        goal_handle = types.SimpleNamespace(
            request=types.SimpleNamespace(msg='need input'),
            abort=self._noop
        )

        with patch.multiple('flexbe_input.input_action_server.Logger',
                            localwarn=self._noop):
            result = server.execute_callback(goal_handle)

        self.assertEqual(BehaviorInput.Result.RESULT_ABORTED, result.result_code)
        self.assertEqual('Another input request is already active!', result.data)
        server._request_lock.release()

    def test_execute_callback_releases_request_lock_on_unsupported_request(self):
        """Request lock should be released on early failure before showing the dialog."""
        aborted = []
        server = InputActionServer.__new__(InputActionServer)
        server._request_lock = threading.Lock()
        server._active_goal_lock = threading.Lock()
        server._active_goal_handle = None
        server._canceled = False
        server.get_input_type = lambda _request_type: None

        goal_handle = types.SimpleNamespace(
            request=types.SimpleNamespace(msg='need input', request_type=255),
            abort=lambda: aborted.append(True)
        )

        with patch.multiple('flexbe_input.input_action_server.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop):
            result = server.execute_callback(goal_handle)

        self.assertEqual(BehaviorInput.Result.RESULT_ABORTED, result.result_code)
        self.assertEqual([True], aborted)
        self.assertTrue(server._request_lock.acquire(blocking=False))
        server._request_lock.release()

    def test_get_input_type_returns_prompt_metadata_for_supported_requests(self):
        """Supported request ids should map to prompt text, accepted type, and element count."""
        server = InputActionServer.__new__(InputActionServer)

        prompt, accepted_type, count = server.get_input_type(BehaviorInput.Goal.REQUEST_3D)

        self.assertEqual('list of 3 numbers', prompt)
        self.assertEqual((list, tuple), accepted_type)
        self.assertEqual(3, count)
        self.assertIsNone(server.get_input_type(999))

    def test_execute_callback_succeeds_for_string_input(self):
        """String requests should return raw text without literal-eval or pickle encoding."""
        succeeded = []
        server = InputActionServer.__new__(InputActionServer)
        server._request_lock = threading.Lock()
        server._active_goal_lock = threading.Lock()
        server._active_goal_handle = None
        server._canceled = False
        server._action_topic = 'flexbe/behavior_input'
        server._worker = types.SimpleNamespace(
            _show_dialog_signal=types.SimpleNamespace(emit=self._noop),
            _hide_dialog_signal=types.SimpleNamespace(emit=self._noop),
        )
        server._input_dialog = types.SimpleNamespace(is_none=lambda: False)
        server._input = 'operator text'

        goal_handle = types.SimpleNamespace(
            request=types.SimpleNamespace(
                msg='need text',
                request_type=BehaviorInput.Goal.REQUEST_STRING,
                items=[],
            ),
            succeed=lambda: succeeded.append(True),
            abort=self._noop,
            canceled=self._noop,
        )

        with patch.multiple('flexbe_input.input_action_server.Logger',
                            localinfo=self._noop,
                            logwarn=self._noop,
                            localwarn=self._noop):
            result = server.execute_callback(goal_handle)

        self.assertEqual([True], succeeded)
        self.assertEqual(BehaviorInput.Result.RESULT_OK, result.result_code)
        self.assertEqual('operator text', result.data)
        self.assertIsNone(server._active_goal_handle)
        self.assertFalse(server._canceled)
        self.assertTrue(server._request_lock.acquire(blocking=False))
        server._request_lock.release()

    def test_execute_callback_rejects_wrong_element_count(self):
        """List requests should fail when the entered sequence length does not match the requested shape."""
        aborted = []
        server = InputActionServer.__new__(InputActionServer)
        server._request_lock = threading.Lock()
        server._active_goal_lock = threading.Lock()
        server._active_goal_handle = None
        server._canceled = False
        server._action_topic = 'flexbe/behavior_input'
        server._worker = types.SimpleNamespace(
            _show_dialog_signal=types.SimpleNamespace(emit=self._noop),
            _hide_dialog_signal=types.SimpleNamespace(emit=self._noop),
        )
        server._input_dialog = types.SimpleNamespace(is_none=lambda: False)
        server._input = '[1, 2]'

        goal_handle = types.SimpleNamespace(
            request=types.SimpleNamespace(
                msg='need pose',
                request_type=BehaviorInput.Goal.REQUEST_3D,
                items=[],
            ),
            succeed=self._noop,
            abort=lambda: aborted.append(True),
            canceled=self._noop,
        )

        with patch.multiple('flexbe_input.input_action_server.Logger',
                            localinfo=self._noop,
                            logwarn=self._noop,
                            localwarn=self._noop):
            result = server.execute_callback(goal_handle)

        self.assertEqual([True], aborted)
        self.assertEqual(BehaviorInput.Result.RESULT_FAILED, result.result_code)
        self.assertIn('Invalid number of elements 2 not 3', result.data)
        self.assertIsNone(server._active_goal_handle)
        self.assertFalse(server._canceled)
        self.assertTrue(server._request_lock.acquire(blocking=False))
        server._request_lock.release()

    def test_execute_callback_aborts_when_request_is_canceled(self):
        """Canceled dialog requests should mark the goal canceled and return an aborted result."""
        canceled = []
        server = InputActionServer.__new__(InputActionServer)
        server._request_lock = threading.Lock()
        server._active_goal_lock = threading.Lock()
        server._active_goal_handle = None
        server._canceled = False
        server._action_topic = 'flexbe/behavior_input'
        server._worker = types.SimpleNamespace(
            _show_dialog_signal=types.SimpleNamespace(emit=lambda prompt, items: setattr(server, '_canceled', True)),
            _hide_dialog_signal=types.SimpleNamespace(emit=self._noop),
        )
        server._input_dialog = types.SimpleNamespace(is_none=lambda: True)
        server._input = None

        goal_handle = types.SimpleNamespace(
            request=types.SimpleNamespace(
                msg='need text',
                request_type=BehaviorInput.Goal.REQUEST_STRING,
                items=[],
            ),
            succeed=self._noop,
            abort=self._noop,
            canceled=lambda: canceled.append(True),
        )

        with patch.multiple('flexbe_input.input_action_server.Logger',
                            localinfo=self._noop,
                            logwarn=self._noop,
                            localwarn=self._noop):
            result = server.execute_callback(goal_handle)

        self.assertEqual([True], canceled)
        self.assertEqual(BehaviorInput.Result.RESULT_ABORTED, result.result_code)
        self.assertEqual('Input request was canceled!', result.data)

    def test_execute_callback_honors_cancel_accepted_before_goal_registration(self):
        """Early accepted cancel should abort before the dialog is shown."""
        canceled = []
        shown = []
        server = InputActionServer.__new__(InputActionServer)
        server._request_lock = threading.Lock()
        server._active_goal_lock = threading.Lock()
        server._active_goal_handle = None
        server._pending_cancel_goal_handle = None
        server._canceled = False
        server._action_topic = 'flexbe/behavior_input'
        server._worker = types.SimpleNamespace(
            _show_dialog_signal=types.SimpleNamespace(emit=lambda *_args: shown.append(True)),
            _hide_dialog_signal=types.SimpleNamespace(emit=self._noop),
        )
        server._input_dialog = types.SimpleNamespace(is_none=lambda: True)
        server._input = None

        goal_handle = types.SimpleNamespace(
            request=types.SimpleNamespace(
                msg='need text',
                request_type=BehaviorInput.Goal.REQUEST_STRING,
                items=[],
            ),
            succeed=self._noop,
            abort=self._noop,
            canceled=lambda: canceled.append(True),
        )

        with patch.multiple('flexbe_input.input_action_server.Logger',
                            localinfo=self._noop,
                            logwarn=self._noop,
                            localwarn=self._noop):
            response = server.cancel_callback(goal_handle)
            result = server.execute_callback(goal_handle)

        self.assertEqual(rclpy.action.CancelResponse.ACCEPT, response)
        self.assertEqual([True], canceled)
        self.assertEqual([], shown)
        self.assertEqual(BehaviorInput.Result.RESULT_ABORTED, result.result_code)
        self.assertEqual('Input request was canceled!', result.data)
        self.assertIsNone(server._active_goal_handle)
        self.assertIsNone(server._pending_cancel_goal_handle)
        self.assertFalse(server._canceled)

    def test_execute_callback_aborts_empty_input_after_dialog_closes(self):
        """Empty dialog submissions should abort instead of returning empty payloads."""
        aborted = []
        server = InputActionServer.__new__(InputActionServer)
        server._request_lock = threading.Lock()
        server._active_goal_lock = threading.Lock()
        server._active_goal_handle = None
        server._canceled = False
        server._action_topic = 'flexbe/behavior_input'
        server._worker = types.SimpleNamespace(
            _show_dialog_signal=types.SimpleNamespace(emit=self._noop),
            _hide_dialog_signal=types.SimpleNamespace(emit=self._noop),
        )
        server._input_dialog = types.SimpleNamespace(is_none=lambda: False)
        server._input = ''

        goal_handle = types.SimpleNamespace(
            request=types.SimpleNamespace(
                msg='need text',
                request_type=BehaviorInput.Goal.REQUEST_STRING,
                items=[],
            ),
            succeed=self._noop,
            abort=lambda: aborted.append(True),
            canceled=self._noop,
        )

        with patch.multiple('flexbe_input.input_action_server.Logger',
                            localinfo=self._noop,
                            logwarn=self._noop,
                            localwarn=self._noop):
            result = server.execute_callback(goal_handle)

        self.assertEqual([True], aborted)
        self.assertEqual(BehaviorInput.Result.RESULT_ABORTED, result.result_code)
        self.assertIn('No data entered', result.data)

    def test_execute_callback_rejects_invalid_literal_type(self):
        """Parsed non-string scalars should be rejected when a sequence type was requested."""
        aborted = []
        server = InputActionServer.__new__(InputActionServer)
        server._request_lock = threading.Lock()
        server._active_goal_lock = threading.Lock()
        server._active_goal_handle = None
        server._canceled = False
        server._action_topic = 'flexbe/behavior_input'
        server._worker = types.SimpleNamespace(
            _show_dialog_signal=types.SimpleNamespace(emit=self._noop),
            _hide_dialog_signal=types.SimpleNamespace(emit=self._noop),
        )
        server._input_dialog = types.SimpleNamespace(is_none=lambda: False)
        server._input = '5'

        goal_handle = types.SimpleNamespace(
            request=types.SimpleNamespace(
                msg='need pose',
                request_type=BehaviorInput.Goal.REQUEST_3D,
                items=[],
            ),
            succeed=self._noop,
            abort=lambda: aborted.append(True),
            canceled=self._noop,
        )

        with patch.multiple('flexbe_input.input_action_server.Logger',
                            localinfo=self._noop,
                            logwarn=self._noop,
                            localwarn=self._noop):
            result = server.execute_callback(goal_handle)

        self.assertEqual([True], aborted)
        self.assertEqual(BehaviorInput.Result.RESULT_FAILED, result.result_code)
        self.assertIn('Invalid input type', result.data)

    def test_cancel_callback_accepts_only_active_goal(self):
        """Cancel should only affect the goal currently owning the dialog."""
        server = InputActionServer.__new__(InputActionServer)
        server._action_topic = 'flexbe/behavior_input'
        server._active_goal_lock = threading.Lock()
        pending_goal = object()
        active_goal = object()
        inactive_goal = object()
        server._canceled = False

        with patch.multiple('flexbe_input.input_action_server.Logger',
                            localwarn=self._noop):
            server._active_goal_handle = None
            response = server.cancel_callback(pending_goal)
            self.assertEqual(rclpy.action.CancelResponse.ACCEPT, response)
            self.assertTrue(server._canceled)
            self.assertIs(pending_goal, server._pending_cancel_goal_handle)

            server._canceled = False
            server._active_goal_handle = active_goal
            response = server.cancel_callback(inactive_goal)
            self.assertEqual(rclpy.action.CancelResponse.REJECT, response)
            self.assertFalse(server._canceled)

            response = server.cancel_callback(active_goal)

        self.assertEqual(rclpy.action.CancelResponse.ACCEPT, response)
        self.assertTrue(server._canceled)
        self.assertIs(active_goal, server._pending_cancel_goal_handle)

    def test_on_get_input_copies_dialog_value(self):
        """The GUI callback should cache the dialog's current input string."""
        server = InputActionServer.__new__(InputActionServer)
        server._input_dialog = types.SimpleNamespace(get_input=lambda: 'operator text')
        server._input = None

        server.on_get_input()

        self.assertEqual(server._input, 'operator text')


if __name__ == '__main__':
    unittest.main()
