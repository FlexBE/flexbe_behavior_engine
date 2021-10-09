#!/usr/bin/env python
import unittest
import rclpy

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from flexbe_core import EventState, OperatableStateMachine
from flexbe_core.core.exceptions import StateError, StateMachineError, UserDataError


class TestExceptions(unittest.TestCase):
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.executor = MultiThreadedExecutor(context=self.context)
        self.node = rclpy.create_node('TestExceptions', context=self.context)

    def tearDown(self):
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)

    def test_invalid_outcome(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        OperatableStateMachine.initialize_ros(self.node)
        node = self.node

        class ReturnInvalidOutcome(EventState):
            def __init__(self):
                self.initialize_ros(node)
                super(ReturnInvalidOutcome, self).__init__(outcomes=['done'])

            def execute(self, userdata):
                return 'invalid'

        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', ReturnInvalidOutcome(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, StateError)

    def test_invalid_transition(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        OperatableStateMachine.initialize_ros(self.node)
        node = self.node

        class ReturnDone(EventState):
            def __init__(self):
                ReturnDone.initialize_ros(node)
                super(ReturnDone, self).__init__(outcomes=['done'])

            def execute(self, userdata):
                return 'done'

        inner_sm = OperatableStateMachine(outcomes=['done'])
        with inner_sm:
            OperatableStateMachine.add('state', ReturnDone(), transitions={'done': 'invalid'})
        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('inner', inner_sm, transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, StateMachineError)

    def test_invalid_userdata_input(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        OperatableStateMachine.initialize_ros(self.node)
        node = self.node

        class AccessInvalidInput(EventState):
            def __init__(self):
                AccessInvalidInput.initialize_ros(node)
                super(AccessInvalidInput, self).__init__(outcomes=['done'], input_keys=['input'])

            def execute(self, userdata):
                print(userdata.invalid)
                return 'done'

        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', AccessInvalidInput(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)

    def test_invalid_userdata_output(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        OperatableStateMachine.initialize_ros(self.node)
        node = self.node

        class SetInvalidOutput(EventState):
            def __init__(self):
                SetInvalidOutput.initialize_ros(node)
                super(SetInvalidOutput, self).__init__(outcomes=['done'], output_keys=['output'])

            def execute(self, userdata):
                userdata.invalid = False
                return 'done'

        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', SetInvalidOutput(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)

    def test_missing_userdata(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        OperatableStateMachine.initialize_ros(self.node)
        node = self.node

        class AccessValidInput(EventState):
            def __init__(self):
                AccessValidInput.initialize_ros(node)
                super(AccessValidInput, self).__init__(outcomes=['done'], input_keys=['missing'])

            def execute(self, userdata):
                print(userdata.missing)
                return 'done'

        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', AccessValidInput(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)

    def test_modify_input_key(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        OperatableStateMachine.initialize_ros(self.node)
        node = self.node

        class ModifyInputKey(EventState):
            def __init__(self):
                ModifyInputKey.initialize_ros(node)
                super(ModifyInputKey, self).__init__(outcomes=['done'], input_keys=['only_input'])

            def execute(self, userdata):
                userdata.only_input['new'] = 'not_allowed'
                return 'done'

        sm = OperatableStateMachine(outcomes=['done'])
        sm.userdata.only_input = {'existing': 'is_allowed'}
        with sm:
            OperatableStateMachine.add('state', ModifyInputKey(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)


if __name__ == '__main__':
    unittest.main()
