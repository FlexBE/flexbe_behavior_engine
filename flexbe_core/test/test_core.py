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


"""Test for flexbe core."""
import time
import unittest

from flexbe_core import ConcurrencyContainer, EventState, OperatableStateMachine, initialize_flexbe_core
from flexbe_core.core import PreemptableState, State
from flexbe_core.core import StateMachineError
from flexbe_core.core import Topics
from flexbe_core.proxy import ProxySubscriberCached, shutdown_proxies

from flexbe_msgs.msg import CommandFeedback, OutcomeRequest

import rclpy
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool, Empty, Int32, UInt32


class CoreTestState(EventState):
    """Test state."""

    _set_state_id = 256

    def __init__(self):
        """Initialize CoreTestState instance."""
        super().__init__(outcomes=['done', 'error'])
        self.result = None
        self.last_events = []
        self.count = 0
        CoreTestState._set_state_id = CoreTestState._set_state_id + 1
        self._state_id = CoreTestState._set_state_id

    def execute(self, userdata):
        """Execute."""
        self.count += 1
        return self.result

    def on_enter(self, userdata):
        """Call on enter."""
        self.last_events.append('on_enter')

    def on_exit(self, userdata):
        """Call on exit."""
        self.last_events.append('on_exit')

    def on_start(self):
        """Call on start."""
        self.last_events.append('on_start')

    def on_stop(self):
        """Call on stop."""
        self.last_events.append('on_stop')

    def on_pause(self):
        """Call on pause."""
        self.last_events.append('on_pause')

    def on_resume(self, userdata):
        """Call on resume."""
        self.last_events.append('on_resume')


class ConcurrencyTestState(CoreTestState):
    """Test state with concurrency debug prints (if needed)."""

    def __init__(self):
        super().__init__()

    # def execute(self, userdata):
    #     outcome = super().execute(userdata)
    #     self._node.get_logger().info('      %s - ConcurrencyTestState execute'
    #                                  '  out=%s count= %d' % (self._name, str(outcome), self.count))
    #     return outcome

    # def on_enter(self, userdata):
    #     self._node.get_logger().info('      %s - ConcurrencyTestState on_enter' % (self._name))
    #     super().on_enter(userdata)

    # def on_exit(self, userdata):
    #     self._node.get_logger().info('      %s - ConcurrencyTestState on_exit count=%d' % (self._name, self.count))
    #     super().on_exit(userdata)


class TestCore(unittest.TestCase):
    """State for core testing."""

    test = 0

    __EXECUTE_TIMEOUT_SEC = 0.2  # 0.025  # Timeout in executor loops for spin once
    __TIME_SLEEP = 0.2  # 0.025  # Sleep time for loops
    __LOOP_COUNT = 50  # Number of times to execute loops for checking (total time ~ LOOP_COUNT*(TIME_SLEEP + TIMEOUT))

    def __init__(self, *args, **kwargs):
        """Initialize TestCore instance."""
        super().__init__(*args, **kwargs)

    def setUp(self):
        """Set up the TestCore test."""
        TestCore.test += 1

        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)

        self.executor = MultiThreadedExecutor(context=self.context)
        self.node = rclpy.create_node('core_test_' + str(self.test), context=self.context)
        # self.node.get_logger().info(' set up core test %d (%d)... ' % (self.test, self.context.ok()))
        self.executor.add_node(self.node)

        initialize_flexbe_core(self.node)

        time.sleep(TestCore.__TIME_SLEEP)

    def tearDown(self):
        """Tear down the TestCore test."""
        self.node.get_logger().info(' shutting down core test %d ... ' % (self.test))
        for _ in range(int(0.25 * TestCore.__LOOP_COUNT)):
            # Allow any lingering pub/sub to clear up
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)

        self.node.get_logger().info('    shutting down proxies in core test %d ... ' % (self.test))
        shutdown_proxies()
        time.sleep(TestCore.__TIME_SLEEP)

        self.node.get_logger().info('    destroy node in core test %d ... ' % (self.test))
        self.node.destroy_node()
        time.sleep(TestCore.__TIME_SLEEP)

        self.executor.shutdown()
        time.sleep(TestCore.__TIME_SLEEP)

        # Kill it with fire to make sure not stray published topics are available
        rclpy.shutdown(context=self.context)
        time.sleep(TestCore.__TIME_SLEEP * 2)

    def _create(self):
        """Create the test."""
        initialize_flexbe_core(self.node)
        state = CoreTestState()
        state._enable_ros_control()
        sm = OperatableStateMachine(outcomes=['done', 'error'])
        sm._state_id = 1024
        sm.set_name('top-level')
        with sm:
            OperatableStateMachine.add('subject', state,
                                       transitions={'done': 'done', 'error': 'error'},
                                       autonomy={'done': 1, 'error': 2})
        return state, sm

    def _execute(self, state):
        """Execute the state via container."""
        # self.node.get_logger().info(' execute %s ... ' % (str(state.name)))
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)

        state.last_events = []
        outcome = state.parent.execute(None)
        # self.node.get_logger().info('       outcome = %s ... ' % (str(outcome)))
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)

        return outcome

    def _reset_sm(self, sm, state):
        """Reset the state machine for another pass."""
        if sm is not None:
            sm._entering = True
            sm.on_enter()
        if state is not None:
            state._entering = True
            state.on_enter(None)

    def _execute_state(self, state):
        """
        Execute the state.

        For this, you may need to call on_enter and on_exit.
        """
        try:
            # self.node.get_logger().info(' execute %s ... ' % (str(state.name)))
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.05)

            state.last_events = []
            outcome = state.execute(None)
            # self.node.get_logger().info('       outcome = %s ... ' % (str(outcome)))
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
            return outcome
        except Exception as exc:
            self.node.get_logger().error(f"    exception in state execute for '{state.name}' ... ")
            self.node.get_logger().info(f'{exc}')
            import traceback
            self.node.get_logger().info(f'{traceback.format_exc()}')

    def assertMessage(self, sub, topic, msg, timeout=1):
        """Check message."""
        for _ in range(int(timeout * TestCore.__LOOP_COUNT)):
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
            if sub.has_msg(topic):
                received = sub.get_last_msg(topic)
                sub.remove_last_msg(topic)
                break
            time.sleep(TestCore.__TIME_SLEEP)
        else:
            raise AssertionError('Did not receive message on topic %s, expected:\n%s'
                                 % (topic, str(msg)))

        if topic in Topics._topic_types:
            self.assertIsInstance(received, Topics.get_type(topic),
                                  f"Failed message type for '{topic}' - was '{type(received)}'"
                                  f" not '{Topics.get_type(topic).__name__}'")
        else:
            print(f"  Unknown topic '{topic}' - not standard FlexBE topic - skip type test!", flush=True)

        for slot in msg.__slots__:
            expected = getattr(msg, slot)
            actual = getattr(received, slot)
            error = "Mismatch for '%s', is %s but expected %s" % (slot, actual, expected)
            if isinstance(expected, list):
                self.assertListEqual(expected, actual, error)
            else:
                self.assertEqual(expected, actual, error)

    def assertNoMessage(self, sub, topic, timeout=1):
        """Assert no message received."""
        if not sub.has_msg(topic):
            self.node.get_logger().info(f"Wait to verify no message arrives for '{topic}' ...")
        for _ in range(int(timeout * TestCore.__LOOP_COUNT)):
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
            if sub.has_msg(topic):
                received = sub.get_last_msg(topic)
                sub.remove_last_msg(topic)
                raise AssertionError('Should not receive message on topic %s, but got:\n%s'
                                     % (topic, str(received)))
            time.sleep(TestCore.__TIME_SLEEP)
        self.node.get_logger().info(f"   No message arrived for '{topic}' - good!")

    # Test Cases
    def test_event_state(self):
        """Test event state."""
        self.node.get_logger().info('test_event_state ...  ')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)

        state, sm = self._create()
        fb_topic = Topics._CMD_FEEDBACK_TOPIC
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        sub = ProxySubscriberCached({fb_topic: CommandFeedback}, inst_id=id(self))
        time.sleep(0.2)

        # enter during first execute
        self._execute(state)
        self.assertListEqual(['on_enter'], state.last_events)

        self._execute(state)
        self.assertListEqual([], state.last_events)

        # pause and resume as commanded
        state._sub._callback(Bool(data=True), Topics._CMD_PAUSE_TOPIC)

        self._execute(state)
        self.assertListEqual(['on_pause'], state.last_events)
        self.assertMessage(sub, fb_topic, CommandFeedback(command='pause'))

        state.result = 'error'
        outcome = self._execute(state)
        state.result = None
        self.assertIsNone(outcome)

        state._sub._callback(Bool(data=False), Topics._CMD_PAUSE_TOPIC)
        self._execute(state)
        self.assertListEqual(['on_resume'], state.last_events)
        self.assertMessage(sub, fb_topic, CommandFeedback(command='resume'))

        # repeat triggers exit and enter again
        state._sub._callback(Empty(), Topics._CMD_REPEAT_TOPIC)
        self._execute(state)
        self.assertListEqual(['on_exit'], state.last_events)
        self.assertMessage(sub, fb_topic, CommandFeedback(command='repeat'))

        self._execute(state)
        self.assertListEqual(['on_enter'], state.last_events)
        self._execute(state)
        self.assertListEqual([], state.last_events)

        # exit during last execute when returning an outcome
        state.result = 'done'
        outcome = self._execute(state)

        self.assertListEqual(['on_exit'], state.last_events)
        self.assertEqual('done', outcome)
        self.node.get_logger().info('test_event_state - OK')

    def test_operatable_state(self):
        """Test operatable state."""
        self.node.get_logger().info('test_operatable_state ... ')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        state, sm = self._create()
        self.node.get_logger().info('test_operatable_state - ProxySubscribe request ...')
        out_topic = Topics._OUTCOME_TOPIC
        req_topic = Topics._OUTCOME_REQUEST_TOPIC
        sub = ProxySubscriberCached({out_topic: UInt32, req_topic: OutcomeRequest}, inst_id=id(self))
        #  wait for pub/sub
        end_time = time.time() + 1
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
            time.sleep(TestCore.__TIME_SLEEP)

        state.on_enter(None)

        # return outcome in full autonomy, no request
        self.node.get_logger().info(f'test_operatable_state - autonomy level={OperatableStateMachine.autonomy_level} '
                                    f'request outcome on full autonomy, no request ...')
        state.result = 'error'
        self._execute_state(state)  # executing single state to we only get the state outputs

        self.assertNoMessage(sub, req_topic)
        self.assertMessage(sub, out_topic, UInt32(data=(2 + state.state_id)))  # Test presumes outcome = 1 + offset

        # request outcome on same autonomy and clear request on loopback
        self.node.get_logger().info('test_operatable_state - request outcome on autonomy level=2 ...')
        self._reset_sm(sm, state)
        OperatableStateMachine.autonomy_level = 2
        self._execute_state(state)  # executing single state to we only get the state outputs
        self.assertNoMessage(sub, out_topic)
        self.assertMessage(sub, req_topic, OutcomeRequest(outcome=1, target=CoreTestState._set_state_id))

        state.result = None
        self._execute_state(state)  # executing single state to we only get the state outputs
        self.assertMessage(sub, req_topic, OutcomeRequest(outcome=255, target=CoreTestState._set_state_id))

        # still return other outcomes
        self.node.get_logger().info('test_operatable_state - still return other outcomes')
        state.result = 'done'
        self._execute_state(state)  # executing single state to we only get the state outputs
        self.assertNoMessage(sub, req_topic)
        self.assertMessage(sub, out_topic, UInt32(data=(1 + state.state_id)))  # Test presumes outcome = 0 + offset

        # request outcome on lower autonomy, return outcome after level increase
        self.node.get_logger().info('test_operatable_state - lower autonomy level=0')
        OperatableStateMachine.autonomy_level = 0
        self._execute_state(state)  # executing single state to we only get the state outputs
        self.assertNoMessage(sub, out_topic)
        self.assertMessage(sub, req_topic, OutcomeRequest(outcome=0, target=CoreTestState._set_state_id))

        OperatableStateMachine.autonomy_level = 3
        self.node.get_logger().info('test_operatable_state -autonomy level=3')
        self._execute_state(state)
        self.assertMessage(sub, out_topic, UInt32(data=(1 + state.state_id)))  # Test presumes outcome = 0 + offset
        self.node.get_logger().info('test_operatable_state - OK! ')

    def test_preemptable_state(self):
        """Test preemptable state."""
        self.node.get_logger().info('test_preemptable_state ... ')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        state, sm = self._create()
        fb_topic = Topics._CMD_FEEDBACK_TOPIC

        self.node.get_logger().info('test_preemptable_state - subscribers ...')

        sub = ProxySubscriberCached({fb_topic: CommandFeedback}, inst_id=id(self))
        #  wait for pub/sub
        end_time = time.time() + 1
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
            time.sleep(TestCore.__TIME_SLEEP)
        # time.sleep(0.2)

        # preempt when trigger variable is set
        PreemptableState.preempt = True
        self.node.get_logger().info('test_preemptable_state - preempt = True ...')
        outcome = self._execute(state)
        self.assertEqual(outcome, State._preempted_name)
        self.assertRaises(StateMachineError, lambda: state.parent.current_state)

        self.node.get_logger().info('test_preemptable_state - preempt = False ...')
        PreemptableState.preempt = False
        outcome = self._execute(state)
        self.assertIsNone(outcome)

        # preempt when command is received
        self.node.get_logger().info('test_preemptable_state - preempt on command ...')
        state._sub._callback(Empty(), Topics._CMD_PREEMPT_TOPIC)
        outcome = self._execute(state)

        self.assertEqual(outcome, State._preempted_name)
        self.assertRaises(StateMachineError, lambda: state.parent.current_state)
        self.assertMessage(sub, fb_topic, CommandFeedback(command='preempt'))

        PreemptableState.preempt = False
        self.node.get_logger().info('test_preemptable_state - OK! ')

    def test_lockable_state(self):
        """Test lockable state."""
        self.node.get_logger().info('test_lockable_state ... ')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        state, sm = self._create()
        fb_topic = Topics._CMD_FEEDBACK_TOPIC
        sub = ProxySubscriberCached({fb_topic: CommandFeedback}, inst_id=id(self))
        time.sleep(0.2)

        # lock and unlock as commanded, return outcome after unlock
        self.node.get_logger().info('  test lock on command ... ')
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        state._sub._callback(Int32(data=state.state_id), Topics._CMD_LOCK_TOPIC)
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        state.result = 'done'

        self.node.get_logger().info('    execute state after lock ... ')
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        outcome = self._execute(state)
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)

        self.node.get_logger().info('    check results of test lock after execute ... ')
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        self.assertIsNone(outcome)
        self.assertTrue(state._locked)
        self.assertMessage(sub, fb_topic, CommandFeedback(command='lock', args=[f'{state.state_id}', f'{state.state_id}']))
        state.result = None

        self.node.get_logger().info('  test unlock on command ... ')
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        state._sub._callback(Int32(data=state.state_id), Topics._CMD_UNLOCK_TOPIC)
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        outcome = self._execute(state)
        self.assertEqual(outcome, 'done')
        self.assertMessage(sub, fb_topic, CommandFeedback(command='unlock', args=[f'{state.state_id}', f'{state.state_id}']))

        # lock and unlock without target
        self.node.get_logger().info('  test lock and unlock without target ... ')
        state._sub._callback(Int32(data=0), Topics._CMD_LOCK_TOPIC)
        state.result = 'done'
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        self.assertMessage(sub, fb_topic, CommandFeedback(command='lock', args=[f'{state.state_id}', f'{state.state_id}']))
        state._sub._callback(Int32(data=0), Topics._CMD_UNLOCK_TOPIC)
        outcome = self._execute(state)
        self.assertEqual(outcome, 'done')
        self.assertMessage(sub, fb_topic, CommandFeedback(command='unlock', args=[f'{state.state_id}', f'{state.state_id}']))

        # reject invalid lock command
        self.node.get_logger().info('  test reject invalid lock command ... ')
        state._sub._callback(Int32(data=12345678), Topics._CMD_LOCK_TOPIC)  # give invalid state id
        outcome = self._execute(state)
        self.assertEqual(outcome, 'done')
        self.assertMessage(sub, fb_topic, CommandFeedback(command='lock', args=['12345678', '-1']))

        # reject generic unlock command when not locked
        self.node.get_logger().info('  test reject invalid unlock when not locked command ... ')
        state._sub._callback(Int32(data=0), Topics._CMD_UNLOCK_TOPIC)
        self._execute(state)
        self.assertMessage(sub, fb_topic, CommandFeedback(command='unlock', args=['0', '-1']))

        # do not transition out of locked container
        self.node.get_logger().info('  test do not transition out of locked container ... ')
        state.parent._locked = True
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        state.parent._locked = False
        state.result = None
        outcome = self._execute(state)
        self.assertEqual(outcome, 'done')
        self.node.get_logger().info('test_lockable_state  - OK! ')

    def test_manually_transitionable_state(self):
        """Test manually transitionable state."""
        self.node.get_logger().info('test_manually_transitionable_state ...')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        state, sm = self._create()
        fb_topic = Topics._CMD_FEEDBACK_TOPIC
        sub = ProxySubscriberCached({fb_topic: CommandFeedback}, inst_id=id(self))
        time.sleep(0.2)

        # return requested outcome
        state._sub._callback(OutcomeRequest(target=CoreTestState._set_state_id, outcome=1), Topics._CMD_TRANSITION_TOPIC)
        outcome = self._execute(state)
        self.assertEqual(outcome, 'error')
        self.assertMessage(sub, fb_topic, CommandFeedback(command='transition',
                                                          args=[f'{CoreTestState._set_state_id}',
                                                                f'{CoreTestState._set_state_id}']))

        # reject outcome request for different target
        state._sub._callback(OutcomeRequest(target=42, outcome=1), Topics._CMD_TRANSITION_TOPIC)
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        # This state won't handle the transition request, so no message is expected
        self.assertNoMessage(sub, fb_topic)
        self.node.get_logger().info('test_manually_transitionable_state - OK! ')

    def test_cross_combinations(self):
        """Test cross combinations."""
        self.node.get_logger().info('test_cross_combinations ...')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        state, sm = self._create()

        # manual transition works on low autonomy
        OperatableStateMachine.autonomy_level = 0
        state.result = 'error'
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        state._sub._callback(OutcomeRequest(target=CoreTestState._set_state_id, outcome=0), Topics._CMD_TRANSITION_TOPIC)
        outcome = self._execute(state)
        self.assertEqual(outcome, 'done')
        OperatableStateMachine.autonomy_level = 3
        state.result = None

        # manual transition blocked by lock
        self.node.get_logger().info('test_cross_combinations - check manual transition blocked by lock ... ')
        state._sub._callback(Int32(data=state.state_id), Topics._CMD_LOCK_TOPIC)
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        state._sub._callback(OutcomeRequest(target=CoreTestState._set_state_id, outcome=1), Topics._CMD_TRANSITION_TOPIC)
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        state._sub._callback(Int32(data=state.state_id), Topics._CMD_UNLOCK_TOPIC)
        outcome = self._execute(state)
        self.assertEqual(outcome, 'error')

        # preempt works on low autonomy
        self.node.get_logger().info('test_cross_combinations - verify preempt works in low autonomy ... ')
        OperatableStateMachine.autonomy_level = 0
        state.result = 'error'
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        state._sub._callback(Empty(), Topics._CMD_PREEMPT_TOPIC)
        outcome = self._execute(state)
        self.assertEqual(outcome, State._preempted_name)
        PreemptableState.preempt = False
        OperatableStateMachine.autonomy_level = 3
        state.result = None

        # preempt also works when locked
        self.node.get_logger().info('test_cross_combinations - verify preempt works when locked ... ')
        state._sub._callback(Int32(data=state.state_id), Topics._CMD_LOCK_TOPIC)
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        state._sub._callback(Empty(), Topics._CMD_PREEMPT_TOPIC)
        outcome = self._execute(state)
        self.assertEqual(outcome, State._preempted_name)
        PreemptableState.preempt = False
        state._sub._callback(Int32(data=state.state_id), Topics._CMD_UNLOCK_TOPIC)
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        self.node.get_logger().info('test_cross_combinations - OK! ')

    def test_concurrency_container(self):
        """Test CC."""
        self.node.get_logger().info('test_concurrency_container ... ')
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)
        cc = ConcurrencyContainer(outcomes=['done', 'error'],
                                  conditions=[
                                  ('error', [('main', 'error')]),
                                  ('error', [('side', 'error')]),
                                  ('done', [('main', 'done'), ('side', 'done')])
                                  ])
        cc._state_id = 2048
        with cc:
            OperatableStateMachine.add('main', ConcurrencyTestState(),
                                       transitions={'done': 'done', 'error': 'error'},
                                       autonomy={'done': 1, 'error': 2})
            OperatableStateMachine.add('side', ConcurrencyTestState(),
                                       transitions={'done': 'done', 'error': 'error'},
                                       autonomy={'done': 1, 'error': 2})
        with OperatableStateMachine(outcomes=['done', 'error']):
            OperatableStateMachine.add('cc', cc,
                                       transitions={'done': 'done', 'error': 'error'},
                                       autonomy={'done': 1, 'error': 2})

        # self.node.get_logger().info('  after setting up OSM - call execute CC ... ')
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)

        # all states are called with their correct rate
        cc.execute(None)

        try:
            self.assertAlmostEqual(cc.sleep_duration, .1, places=2)
        except AssertionError:  # pylint: disable=W0703
            self.node.get_logger().warn(f' Caught error with cc.sleep_duration = {cc.sleep_duration:.6f} =/= 0.1 '
                                        f'- Sometimes fails if OS interruption! ... ')

        # Not controlled yet (e.g. as if no UI connected)
        self.assertFalse(cc['main']._is_controlled)
        self.assertFalse(cc['side']._is_controlled)

        # cc.sleep()
        cc['main'].set_rate(15)
        cc['side'].set_rate(10)
        cc['main'].count = 0
        cc['side'].count = 0

        cc_count = 0
        test_start_time = self.node.get_clock().now()
        start_time = time.time()
        elapsed = None
        self.node.get_logger().info('  test timing loop  ... ')
        while (self.node.get_clock().now() - test_start_time).nanoseconds < 1000000001:  # 1 second
            cc_count += 1
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.0005)
            outcome = cc.execute(None)
            self.node.get_logger().info('  cc_count=%d (%d, %d) outcome=%s duration=%s (%s, %s) last= (%s, %s) now=%s' %
                                        (cc_count,
                                         cc['main'].count, cc['side'].count, str(outcome),
                                         str(cc.sleep_duration),
                                         str(cc['main'].sleep_duration), str(cc['side'].sleep_duration),
                                         str(cc['main']._last_execution.nanoseconds), str(cc['side']._last_execution.nanoseconds),
                                         str(self.node.get_clock().now().nanoseconds)
                                         ))
            self.assertLessEqual(cc.sleep_duration, .1)
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.0005)
            elapsed = time.time() - start_time  # actual time for iteration
            start_time = time.time()
            elapsed = 0.1 - elapsed
            sleep_time = min(elapsed, cc.sleep_duration)
            if sleep_time > 0.00005:
                time.sleep(sleep_time)

        self.assertIn(cc['main'].count, [14, 15, 16])
        self.assertIn(cc['side'].count, [9, 10, 11])
        self.assertLessEqual(cc_count, 30)  # 1 / (1/10 - 1/15)

        self.node.get_logger().info('  verify ROS properties  ... ')

        # verify ROS properties
        cc._enable_ros_control()
        self.assertTrue(cc['main']._is_controlled)
        self.assertTrue(cc['side']._is_controlled)  # New change!

        # return outcome when all return done or any returns error
        cc['main'].set_rate(1.e16)  # run every time
        cc['side'].set_rate(1.e16)  # run every time

        self.node.get_logger().info('  verify outcomes ... ')
        outcome = cc.execute(None)
        self.assertIsNone(outcome)

        cc['main'].result = 'error'
        outcome = cc.execute(None)
        self.assertEqual(outcome, 'error')

        cc['main'].result = None
        cc['side'].result = 'error'
        outcome = cc.execute(None)
        self.assertEqual(outcome, 'error')

        self.node.get_logger().info('  verify outcome only if both done (as set by conditions above) ... ')
        cc['main'].result = None
        cc['side'].result = None
        outcome = cc.execute(None)
        self.assertIsNone(outcome)

        cc['main'].result = None
        cc['side'].result = 'done'
        outcome = cc.execute(None)
        self.assertIsNone(outcome)

        cc['main'].result = 'done'
        self.node.get_logger().info('  verify  both done returns outcome(as set by conditions above) ... ')
        outcome = cc.execute(None)
        self.assertEqual(outcome, 'done')

        # always call on_exit exactly once when returning an outcome
        self.node.get_logger().info('  verify on_exit called once and only once  ... ')
        cc['main'].result = None
        cc['side'].result = None
        outcome = cc.execute(None)
        self.assertIsNone(outcome)

        self.node.get_logger().info('  check last events ... ')
        cc['main'].last_events = []
        cc['side'].last_events = []
        cc['main'].result = 'error'
        outcome = cc.execute(None)
        self.assertEqual(outcome, 'error')
        self.assertListEqual(cc['main'].last_events, ['on_exit'])
        self.assertListEqual(cc['side'].last_events, ['on_exit'])

        self.node.get_logger().info('test_concurrency_container - OK! ')

    def test_user_data(self):
        """Test user data."""
        self.node.get_logger().info('test_user_data ...')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestCore.__EXECUTE_TIMEOUT_SEC)

        class TestUserdataState(EventState):
            """Local Test state."""

            def __init__(self, out_content='test_data'):
                super(TestUserdataState, self).__init__(outcomes=['done'], input_keys=['data_in'], output_keys=['data_out'])
                self.data = None
                self._out_content = out_content
                CoreTestState._set_state_id = CoreTestState._set_state_id + 1
                self._state_id = CoreTestState._set_state_id

            def execute(self, userdata):
                self._node.get_logger().warn('\033[0m%s\n%s' % (self.path, str(userdata)))  # log for manual inspection
                self.data = userdata.data_in
                userdata.data_out = self._out_content
                return 'done'

        inner_sm = OperatableStateMachine(outcomes=['done'], input_keys=['sm_in'], output_keys=['sm_out'])
        inner_sm._state_id = 4096
        inner_sm.userdata.own = 'own_data'
        with inner_sm:
            OperatableStateMachine.add('own_state', TestUserdataState('inner_data'), transitions={'done': 'outside_state'},
                                       remapping={'data_in': 'own', 'data_out': 'sm_out'},
                                       autonomy={'done': 0})
            OperatableStateMachine.add('outside_state', TestUserdataState(), transitions={'done': 'internal_state'},
                                       remapping={'data_in': 'sm_in', 'data_out': 'data_in'},
                                       autonomy={'done': 0})
            OperatableStateMachine.add('internal_state', TestUserdataState(), transitions={'done': 'done'},
                                       remapping={},
                                       autonomy={'done': 0})

        sm = OperatableStateMachine(outcomes=['done'])
        sm._state_id = 8192
        sm.userdata.outside = 'outside_data'
        sm.set_name('outer_sm')  # No one sets this name, while .add sets names of inner states
        with sm:
            OperatableStateMachine.add('before_state', TestUserdataState(), transitions={'done': 'inner_sm'},
                                       remapping={'data_in': 'outside'},
                                       autonomy={'done': 0})
            OperatableStateMachine.add('inner_sm', inner_sm, transitions={'done': 'after_state'},
                                       remapping={'sm_in': 'outside'},
                                       autonomy={'done': 0})
            OperatableStateMachine.add('after_state', TestUserdataState('last_data'), transitions={'done': 'modify_state'},
                                       remapping={'data_in': 'sm_out'},
                                       autonomy={'done': 0})
            OperatableStateMachine.add('modify_state', TestUserdataState(), transitions={'done': 'final_state'},
                                       remapping={'data_out': 'outside', 'data_in': 'outside'},
                                       autonomy={'done': 0})
            OperatableStateMachine.add('final_state', TestUserdataState(), transitions={'done': 'done'},
                                       remapping={'data_in': 'data_out'},
                                       autonomy={'done': 0})

        # can pass userdata to state and get it from state
        sm.execute(None)
        self.assertEqual(sm['before_state'].data, 'outside_data')
        self.assertEqual(sm._userdata.data_out, 'test_data')

        # sub-state machine can set its own local userdata
        sm.execute(None)
        self.assertEqual(sm['inner_sm']['own_state'].data, 'own_data')
        self.assertNotIn('own', sm._userdata)  # transparent to outer sm

        # sub-state machine can read data from parent state machine
        sm.execute(None)
        self.assertEqual(sm['inner_sm']['outside_state'].data, 'outside_data')

        # sub-state machine can pass along its local userdata
        self.assertIn('data_in', sm['inner_sm']._userdata)
        sm.execute(None)
        self.assertEqual(sm['inner_sm']['internal_state'].data, 'test_data')
        self.assertNotIn('data_in', sm._userdata)  # transparent to outer sm

        # sub-state machine userdata is wrote back to the parent
        self.assertEqual(sm._userdata.sm_out, 'inner_data')

        # outer state machine can read data set by inner state machine
        sm.execute(None)
        self.assertEqual(sm['after_state'].data, 'inner_data')

        # can remap different keys to achieve read-write access
        sm.execute(None)
        self.assertEqual(sm['modify_state'].data, 'outside_data')
        self.assertEqual(sm._userdata.outside, 'test_data')

        # one state can read data set by another one
        outcome = sm.execute(None)
        self.assertEqual(sm['final_state'].data, 'last_data')
        self.assertEqual(outcome, 'done')
        self.node.get_logger().info('test_user_data - OK! ')


if __name__ == '__main__':
    unittest.main()
