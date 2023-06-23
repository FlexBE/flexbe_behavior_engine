#!/usr/bin/env python3

# Copyright 2023 Philipp Schillinger, Team ViGIR, Christopher Newport University
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


"""Test FlexBE Exception handling."""
import time
import unittest

import rclpy

from rclpy.executors import MultiThreadedExecutor
from flexbe_core import EventState, OperatableStateMachine
from flexbe_core.core.exceptions import StateError, StateMachineError, UserDataError
from flexbe_core.proxy import initialize_proxies, shutdown_proxies


class TestExceptions(unittest.TestCase):
    """Test FlexBE Exception handling."""

    test = 0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def setUp(self):
        TestExceptions.test += 1
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)

        self.executor = MultiThreadedExecutor(context=self.context)
        self.node = rclpy.create_node("exception_test_" + str(self.test), context=self.context)
        self.node.get_logger().info(" set up exceptions test %d (%d) ... " % (self.test, self.context.ok()))
        self.executor.add_node(self.node)
        initialize_proxies(self.node)

    def tearDown(self):
        self.node.get_logger().info(" shutting down exceptions test %d (%d) ... " % (self.test, self.context.ok()))
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

        self.node.get_logger().info("    shutting down proxies in core test %d ... " % (self.test))
        shutdown_proxies()
        time.sleep(0.1)

        self.node.get_logger().info("    destroy node in core test %d ... " % (self.test))
        self.node.destroy_node()

        time.sleep(0.1)
        self.executor.shutdown()
        time.sleep(0.1)

        # Kill it with fire to make sure not stray published topics are available
        rclpy.shutdown(context=self.context)
        time.sleep(0.2)

    def test_invalid_outcome(self):
        self.node.get_logger().info("test_invalid_outcome ...")

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        OperatableStateMachine.initialize_ros(self.node)
        node = self.node

        class ReturnInvalidOutcomeState(EventState):
            """Local Test state definition."""

            def __init__(self):
                self.initialize_ros(node)
                super().__init__(outcomes=['done'])

            def execute(self, userdata):
                return 'invalid'

        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', ReturnInvalidOutcomeState(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, StateError)
        self.node.get_logger().info("test_invalid_outcome  - OK! ")

    def test_invalid_transition(self):
        self.node.get_logger().info("test_invalid_transition ...")

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        OperatableStateMachine.initialize_ros(self.node)
        node = self.node

        class ReturnDoneState(EventState):
            """Local Test state definition."""

            def __init__(self):
                ReturnDoneState.initialize_ros(node)
                super().__init__(outcomes=['done'])

            def execute(self, userdata):
                return 'done'

        inner_sm = OperatableStateMachine(outcomes=['done'])
        with inner_sm:
            OperatableStateMachine.add('state', ReturnDoneState(), transitions={'done': 'invalid'})
        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('inner', inner_sm, transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, StateMachineError)
        self.node.get_logger().info("test_invalid_transition - OK! ")

    def test_invalid_userdata_input(self):
        self.node.get_logger().info("test_invalid_userdata ...")

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        OperatableStateMachine.initialize_ros(self.node)
        node = self.node

        class AccessInvalidInputState(EventState):
            """Local Test state definition."""

            def __init__(self):
                AccessInvalidInputState.initialize_ros(node)
                super().__init__(outcomes=['done'], input_keys=['input'])

            def execute(self, userdata):
                print(userdata.invalid)
                return 'done'

        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', AccessInvalidInputState(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)
        self.node.get_logger().info("test_invalid_userdata - OK! ")

    def test_invalid_userdata_output(self):
        self.node.get_logger().info("test_invalid_userdata_output ...")

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        OperatableStateMachine.initialize_ros(self.node)
        node = self.node

        class SetInvalidOutputState(EventState):
            """Local Test state definition."""

            def __init__(self):
                SetInvalidOutputState.initialize_ros(node)
                super().__init__(outcomes=['done'], output_keys=['output'])

            def execute(self, userdata):
                userdata.invalid = False
                return 'done'

        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', SetInvalidOutputState(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)
        self.node.get_logger().info("test_invalid_userdata_output - OK! ")

    def test_missing_userdata(self):
        self.node.get_logger().info("test_missing_userdata ...")

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        OperatableStateMachine.initialize_ros(self.node)
        node = self.node

        class AccessValidInputState(EventState):
            """Local Test state definition."""

            def __init__(self):
                AccessValidInputState.initialize_ros(node)
                super().__init__(outcomes=['done'], input_keys=['missing'])

            def execute(self, userdata):
                print(userdata.missing)
                return 'done'

        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', AccessValidInputState(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)
        self.node.get_logger().info("test_missing_userdata - OK! ")

    def test_modify_input_key(self):
        self.node.get_logger().info("test_modify_input_key ...! ")

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        OperatableStateMachine.initialize_ros(self.node)
        node = self.node

        class ModifyInputKeyState(EventState):
            """Local Test state definition."""

            def __init__(self):
                ModifyInputKeyState.initialize_ros(node)
                super().__init__(outcomes=['done'], input_keys=['only_input'])

            def execute(self, userdata):
                userdata.only_input['new'] = 'not_allowed'
                return 'done'

        sm = OperatableStateMachine(outcomes=['done'])
        sm.userdata.only_input = {'existing': 'is_allowed'}
        with sm:
            OperatableStateMachine.add('state', ModifyInputKeyState(), transitions={'done': 'done'})

        outcome = sm.execute(None)
        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)
        self.node.get_logger().info("test_modify_input_key - OK! ")


if __name__ == '__main__':
    unittest.main()
