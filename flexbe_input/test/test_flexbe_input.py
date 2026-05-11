#!/usr/bin/env python3

"""Unit tests for FlexBEInput relay behavior."""

import types
import unittest
from unittest.mock import patch

from flexbe_input.flexbe_input import FlexBEInput

from flexbe_msgs.action import BehaviorInput


class TestFlexBEInput(unittest.TestCase):
    """Validate local relay behavior without ROS startup."""

    @staticmethod
    def _noop(*args, **kwargs):
        return None

    def test_init_creates_complex_action_server_and_sets_timeout(self):
        """Construction should create the relay action server and advertise readiness."""
        fake_node = object()
        created = []

        def _fake_action_server(**kwargs):
            created.append(kwargs)
            return 'action-server'

        with patch('flexbe_input.flexbe_input.ComplexActionServer', side_effect=_fake_action_server), \
                patch('flexbe_input.flexbe_input.Logger.loginfo') as loginfo:
            be_input = FlexBEInput(fake_node)

        self.assertIs(be_input._node, fake_node)
        self.assertEqual(be_input._as, 'action-server')
        self.assertEqual(be_input._server_wait_timeout, 1.0)
        self.assertEqual(created[0]['node'], fake_node)
        self.assertEqual(created[0]['name'], 'flexbe/behavior_input')
        self.assertIs(created[0]['ActionSpec'], BehaviorInput)
        self.assertEqual(created[0]['execute_cb'].__self__, be_input)
        self.assertEqual(created[0]['execute_cb'].__func__, FlexBEInput.execute_cb)
        self.assertFalse(created[0]['auto_start'])
        loginfo.assert_called_once_with('Ready for data requests...')

    def test_execute_cb_aborts_when_operator_server_is_unavailable(self):
        """Unavailable operator input server should abort quickly instead of hanging."""
        completed = []
        relay_client = types.SimpleNamespace(
            wait_for_server=lambda timeout_sec=None: False
        )
        be_input = FlexBEInput.__new__(FlexBEInput)
        be_input._node = object()
        be_input._server_wait_timeout = 0.5
        be_input._as = types.SimpleNamespace(
            set_aborted=lambda result, text, goal_handle: completed.append((result, text, goal_handle))
        )

        goal = types.SimpleNamespace(msg='need input')
        goal_handle = object()

        with patch('flexbe_input.flexbe_input.ActionClient', return_value=relay_client):
            with patch.multiple('flexbe_input.flexbe_input.Logger',
                                loginfo=self._noop,
                                localinfo=self._noop,
                                logwarn=self._noop):
                be_input.execute_cb(goal, goal_handle)

        self.assertEqual(1, len(completed))
        result, text, handle = completed[0]
        self.assertEqual(BehaviorInput.Result.RESULT_ABORTED, result.result_code)
        self.assertEqual('Timed out waiting for flexbe/operator_input action server', result.data)
        self.assertEqual('Timed out waiting for operator input server', text)
        self.assertIs(goal_handle, handle)

    def test_execute_cb_relay_success_returns_ok_result(self):
        """Successful operator relay should forward the returned payload unchanged."""
        completed = []
        relay_result = BehaviorInput.Result(
            result_code=BehaviorInput.Result.RESULT_OK,
            data='serialized-payload',
        )
        relay_client = types.SimpleNamespace(
            wait_for_server=lambda timeout_sec=None: True,
            send_goal=lambda goal: relay_result,
        )
        be_input = FlexBEInput.__new__(FlexBEInput)
        be_input._node = object()
        be_input._server_wait_timeout = 0.5
        be_input._as = types.SimpleNamespace(
            set_succeeded=lambda result, text, goal_handle: completed.append((result, text, goal_handle))
        )

        goal = types.SimpleNamespace(msg='need input')
        goal_handle = object()

        with patch('flexbe_input.flexbe_input.ActionClient', return_value=relay_client), \
                patch.multiple('flexbe_input.flexbe_input.Logger',
                               loginfo=self._noop,
                               localinfo=self._noop,
                               logwarn=self._noop):
            be_input.execute_cb(goal, goal_handle)

        self.assertEqual(1, len(completed))
        result, text, handle = completed[0]
        self.assertEqual(BehaviorInput.Result.RESULT_OK, result.result_code)
        self.assertEqual('serialized-payload', result.data)
        self.assertEqual('ok', text)
        self.assertIs(goal_handle, handle)

    def test_execute_cb_relay_failure_and_abort_use_matching_result_codes(self):
        """Non-OK relay results should be forwarded with their original payload and completion text."""
        for result_code, expected_text in [
            (BehaviorInput.Result.RESULT_FAILED, 'failed'),
            (BehaviorInput.Result.RESULT_ABORTED, 'Aborted'),
        ]:
            completed = []
            relay_result = BehaviorInput.Result(result_code=result_code, data=f'payload-{result_code}')
            relay_client = types.SimpleNamespace(
                wait_for_server=lambda timeout_sec=None: True,
                send_goal=lambda goal, relay_result=relay_result: relay_result,
            )
            be_input = FlexBEInput.__new__(FlexBEInput)
            be_input._node = object()
            be_input._server_wait_timeout = 0.5
            be_input._as = types.SimpleNamespace(
                set_succeeded=lambda result, text, goal_handle: completed.append((result, text, goal_handle)),
                set_aborted=lambda result, text, goal_handle: completed.append((result, text, goal_handle)),
            )

            goal = types.SimpleNamespace(msg='need input')
            goal_handle = object()

            with patch('flexbe_input.flexbe_input.ActionClient', return_value=relay_client), \
                    patch.multiple('flexbe_input.flexbe_input.Logger',
                                   loginfo=self._noop,
                                   localinfo=self._noop,
                                   logwarn=self._noop):
                be_input.execute_cb(goal, goal_handle)

            self.assertEqual(1, len(completed))
            result, text, handle = completed[0]
            self.assertEqual(result_code, result.result_code)
            self.assertEqual(f'payload-{result_code}', result.data)
            self.assertEqual(expected_text, text)
            self.assertIs(goal_handle, handle)

    def test_execute_cb_ignores_unrecognized_result_codes(self):
        """Unexpected relay result codes should not fabricate a success response."""
        completed = []
        relay_result = BehaviorInput.Result(result_code=255, data='mystery')
        relay_client = types.SimpleNamespace(
            wait_for_server=lambda timeout_sec=None: True,
            send_goal=lambda goal: relay_result,
        )
        be_input = FlexBEInput.__new__(FlexBEInput)
        be_input._node = object()
        be_input._server_wait_timeout = 0.5
        be_input._as = types.SimpleNamespace(
            set_succeeded=lambda result, text, goal_handle: completed.append((result, text, goal_handle))
        )

        goal = types.SimpleNamespace(msg='need input')
        goal_handle = object()

        with patch('flexbe_input.flexbe_input.ActionClient', return_value=relay_client), \
                patch.multiple('flexbe_input.flexbe_input.Logger',
                               loginfo=self._noop,
                               localinfo=self._noop,
                               logwarn=self._noop):
            result = be_input.execute_cb(goal, goal_handle)

        self.assertIsNone(result)
        self.assertEqual(completed, [])

    def test_main_spins_and_shuts_down_node_in_finally(self):
        """The module entry point should always destroy the node and try shutting down rclpy."""
        destroyed = []
        node = types.SimpleNamespace(destroy_node=lambda: destroyed.append(True))

        with patch('flexbe_input.flexbe_input.rclpy.init') as rclpy_init, \
                patch('flexbe_input.flexbe_input.rclpy.create_node', return_value=node) as create_node, \
                patch('flexbe_input.flexbe_input.FlexBEInput') as flexbe_input_cls, \
                patch('flexbe_input.flexbe_input.rclpy.spin', side_effect=RuntimeError('stop spin')), \
                patch('flexbe_input.flexbe_input.rclpy.try_shutdown') as try_shutdown:
            with self.assertRaisesRegex(RuntimeError, 'stop spin'):
                from flexbe_input.flexbe_input import main
                main(args=['--ros-args'])

        rclpy_init.assert_called_once_with(args=['--ros-args'])
        create_node.assert_called_once_with('flexbe_input')
        flexbe_input_cls.assert_called_once_with(node)
        self.assertEqual(destroyed, [True])
        try_shutdown.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
