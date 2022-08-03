#!/usr/bin/env python
import unittest
import time
import threading
import rclpy

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from flexbe_core import EventState, OperatableStateMachine, ConcurrencyContainer
from flexbe_core.core import PreemptableState
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.core.exceptions import StateMachineError

from std_msgs.msg import Bool, Empty, UInt8, String
from flexbe_msgs.msg import CommandFeedback, OutcomeRequest


class SubjectStateTest(EventState):

    def __init__(self):
        super(SubjectStateTest, self).__init__(outcomes=['done', 'error'])
        self.result = None
        self.last_events = []
        self.count = 0

    def execute(self, userdata):
        self.count += 1
        return self.result

    def on_enter(self, userdata):
        self.last_events.append('on_enter')

    def on_exit(self, userdata):
        self.last_events.append('on_exit')

    def on_start(self):
        self.last_events.append('on_start')

    def on_stop(self):
        self.last_events.append('on_stop')

    def on_pause(self):
        self.last_events.append('on_pause')

    def on_resume(self, userdata):
        self.last_events.append('on_resume')


class TestCore(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestCore, self).__init__(*args, **kwargs)
        self.test = 0

    @classmethod
    def setUp(self):
        try:
            self.test += 1
        except:
            self.test = 0

        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.executor = MultiThreadedExecutor(context=self.context)
        self.node = rclpy.create_node("test" + str(self.test), context=self.context)

    # @classmethod
    # def tearDown(self):
    #     self.node.destroy_node()
    #     self.executor.shutdown()
    #     rclpy.shutdown(context=self.context)

    def _create(self):
        SubjectStateTest.initialize_ros(self.node)
        OperatableStateMachine.initialize_ros(self.node)
        state = SubjectStateTest()
        state._enable_ros_control()
        with OperatableStateMachine(outcomes=['done', 'error']):
            OperatableStateMachine.add('subject', state,
                                       transitions={'done': 'done', 'error': 'error'},
                                       autonomy={'done': 1, 'error': 2})
        return state

    def _execute(self, state):
        state.last_events = []
        return state.parent.execute(None)

    def assertMessage(self, sub, topic, msg, timeout=1):
        for i in range(int(timeout * 100)):
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
            if sub.has_msg(topic):
                received = sub.get_last_msg(topic)
                sub.remove_last_msg(topic)
                break
            time.sleep(0.1)
        else:
            raise AssertionError('Did not receive message on topic %s, expected:\n%s'
                                 % (topic, str(msg)))
        for slot in msg.__slots__:
            expected = getattr(msg, slot)
            actual = getattr(received, slot)
            error = "Mismatch for %s, is %s but expected %s" % (slot, actual, expected)
            if isinstance(expected, list):
                self.assertListEqual(expected, actual, error)
            else:
                self.assertEqual(expected, actual, error)

    def assertNoMessage(self, sub, topic, timeout=1):
        rate = self.node.create_rate(100, self.node.get_clock())
        for i in range(int(timeout * 100)):
            if sub.has_msg(topic):
                received = sub.get_last_msg(topic)
                sub.remove_last_msg(topic)
                raise AssertionError('Should not receive message on topic %s, but got:\n%s'
                                     % (topic, str(received)))

    # Test Cases
    def test_event_state(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        ProxySubscriberCached._initialize(self.node)
        state = self._create()
        fb_topic = 'flexbe/command_feedback'
        sub = ProxySubscriberCached({fb_topic: CommandFeedback}, id=id(self))
        time.sleep(0.2)

        # enter during first execute
        self._execute(state)
        self.assertListEqual(['on_enter'], state.last_events)
        self._execute(state)
        self.assertListEqual([], state.last_events)

        # pause and resume as commanded
        state._sub._callback(Bool(data=True), 'flexbe/command/pause')

        self._execute(state)
        self.assertListEqual(['on_pause'], state.last_events)
        self.assertMessage(sub, fb_topic, CommandFeedback(command="pause"))

        state.result = 'error'
        outcome = self._execute(state)
        state.result = None
        self.assertIsNone(outcome)

        state._sub._callback(Bool(data=False), 'flexbe/command/pause')
        self._execute(state)
        self.assertListEqual(['on_resume'], state.last_events)
        self.assertMessage(sub, fb_topic, CommandFeedback(command="resume"))

        # repeat triggers exit and enter again
        state._sub._callback(Empty(), 'flexbe/command/repeat')
        self._execute(state)
        self.assertListEqual(['on_exit'], state.last_events)
        self.assertMessage(sub, fb_topic, CommandFeedback(command="repeat"))

        self._execute(state)
        self.assertListEqual(['on_enter'], state.last_events)
        self._execute(state)
        self.assertListEqual([], state.last_events)

        # exit during last execute when returning an outcome
        state.result = 'done'
        outcome = self._execute(state)

        self.assertListEqual(['on_exit'], state.last_events)
        self.assertEqual('done', outcome)

    def test_operatable_state(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        ProxySubscriberCached._initialize(self.node)
        state = self._create()
        out_topic = 'flexbe/mirror/outcome'
        req_topic = 'flexbe/outcome_request'
        sub = ProxySubscriberCached({out_topic: UInt8, req_topic: OutcomeRequest}, id=id(self))
        #  wait for pub/sub
        end_time = time.time() + 1
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
            time.sleep(0.1)

        # return outcome in full autonomy, no request
        state.result = 'error'
        self._execute(state)

        self.assertNoMessage(sub, req_topic)
        self.assertMessage(sub, out_topic, UInt8(data=1))

        # request outcome on same autnomy and clear request on loopback
        OperatableStateMachine.autonomy_level = 2
        self._execute(state)
        self.assertNoMessage(sub, out_topic)
        self.assertMessage(sub, req_topic, OutcomeRequest(outcome=1, target='/subject'))

        state.result = None
        self._execute(state)
        self.assertMessage(sub, req_topic, OutcomeRequest(outcome=255, target='/subject'))

        # still return other outcomes
        state.result = 'done'
        self._execute(state)
        self.assertNoMessage(sub, req_topic)
        self.assertMessage(sub, out_topic, UInt8(data=0))

        # request outcome on lower autonomy, return outcome after level increase
        OperatableStateMachine.autonomy_level = 0
        self._execute(state)
        self.assertNoMessage(sub, out_topic)
        self.assertMessage(sub, req_topic, OutcomeRequest(outcome=0, target='/subject'))

        OperatableStateMachine.autonomy_level = 3
        self._execute(state)
        self.assertMessage(sub, out_topic, UInt8(data=0))

    def test_preemptable_state(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        ProxySubscriberCached._initialize(self.node)
        state = self._create()
        fb_topic = 'flexbe/command_feedback'
        sub = ProxySubscriberCached({fb_topic: CommandFeedback}, id=id(self))
        #  wait for pub/sub
        end_time = time.time() + 1
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
            time.sleep(0.1)
        # time.sleep(0.2)

        # preempt when trigger variable is set
        PreemptableState.preempt = True
        outcome = self._execute(state)
        self.assertEqual(outcome, PreemptableState._preempted_name)
        self.assertRaises(StateMachineError, lambda: state.parent.current_state)
        PreemptableState.preempt = False
        outcome = self._execute(state)
        self.assertIsNone(outcome)

        # preempt when command is received
        state._sub._callback(Empty(), 'flexbe/command/preempt')
        outcome = self._execute(state)
        self.assertEqual(outcome, PreemptableState._preempted_name)
        self.assertRaises(StateMachineError, lambda: state.parent.current_state)
        self.assertMessage(sub, fb_topic, CommandFeedback(command='preempt'))
        PreemptableState.preempt = False

    def test_lockable_state(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        ProxySubscriberCached._initialize(self.node)
        state = self._create()
        fb_topic = 'flexbe/command_feedback'
        sub = ProxySubscriberCached({fb_topic: CommandFeedback}, id=id(self))
        time.sleep(0.2)

        # lock and unlock as commanded, return outcome after unlock
        state._sub._callback(String(data='/subject'), 'flexbe/command/lock')
        state.result = 'done'
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        self.assertTrue(state._locked)
        self.assertMessage(sub, fb_topic, CommandFeedback(command='lock', args=['/subject', '/subject']))
        state.result = None
        state._sub._callback(String(data='/subject'), 'flexbe/command/unlock')
        outcome = self._execute(state)
        self.assertEqual(outcome, 'done')
        self.assertMessage(sub, fb_topic, CommandFeedback(command='unlock', args=['/subject', '/subject']))

        # lock and unlock without target
        state._sub._callback(String(data=''), 'flexbe/command/lock')
        state.result = 'done'
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        self.assertMessage(sub, fb_topic, CommandFeedback(command='lock', args=['/subject', '/subject']))
        state._sub._callback(String(data=''), 'flexbe/command/unlock')
        outcome = self._execute(state)
        self.assertEqual(outcome, 'done')
        self.assertMessage(sub, fb_topic, CommandFeedback(command='unlock', args=['/subject', '/subject']))

        # reject invalid lock command
        state._sub._callback(String(data='/invalid'), 'flexbe/command/lock')
        outcome = self._execute(state)
        self.assertEqual(outcome, 'done')
        self.assertMessage(sub, fb_topic, CommandFeedback(command='lock', args=['/invalid', '']))

        # reject generic unlock command when not locked
        state._sub._callback(String(data=''), 'flexbe/command/unlock')
        self._execute(state)
        self.assertMessage(sub, fb_topic, CommandFeedback(command='unlock', args=['', '']))

        # do not transition out of locked container
        state.parent._locked = True
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        state.parent._locked = False
        state.result = None
        outcome = self._execute(state)
        self.assertEqual(outcome, 'done')

    def test_manually_transitionable_state(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        ProxySubscriberCached._initialize(self.node)
        state = self._create()
        fb_topic = 'flexbe/command_feedback'
        sub = ProxySubscriberCached({fb_topic: CommandFeedback}, id=id(self))
        time.sleep(0.2)

        # return requested outcome
        state._sub._callback(OutcomeRequest(target='subject', outcome=1), 'flexbe/command/transition')
        outcome = self._execute(state)
        self.assertEqual(outcome, 'error')
        self.assertMessage(sub, fb_topic, CommandFeedback(command='transition', args=['subject', 'subject']))

        # reject outcome request for different target
        state._sub._callback(OutcomeRequest(target='invalid', outcome=1), 'flexbe/command/transition')
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        self.assertMessage(sub, fb_topic, CommandFeedback(command='transition', args=['invalid', 'subject']))


    # Problem with RosState getting locked on state.sleep()
    # def test_ros_state(self):
    #     # rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
    #     state = self._create()
    #
    #     # default rate is 10Hz
    #     start = self.node.get_clock().now()
    #     for i in range(10):
    #         rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
    #         state.sleep()
    #     duration = (self.node.get_clock().now() - start).nanoseconds
    #     duration = duration * 10 ** -9
    #     self.assertAlmostEqual(duration, 1.0, places=2)
    #
    #     sleep_duration = state.sleep_duration
    #     sleep_duration = sleep_duration * 10 ** -9
    #     self.assertAlmostEqual(sleep_duration, .1, places=2)
    #
    #     # change of rate works as expected
    #     state.set_rate(1)
    #     start = self.node.get_clock().now()
    #     rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
    #     print("Getting ready to sleep")
    #     state.sleep()
    #     print("Done sleeping")
    #
    #     duration = (self.node.get_clock().now() - start).nanoseconds
    #     duration = duration * 10 ** -9
    #     self.assertAlmostEqual(duration, 1., places=2)
    #
    #     sleep_duration = state.sleep_duration
    #     sleep_duration = sleep_duration * 10 ** -9
    #     self.assertAlmostEqual(sleep_duration, 1., places=2)


    def test_cross_combinations(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        state = self._create()

        # manual transition works on low autonomy
        OperatableStateMachine.autonomy_level = 0
        state.result = 'error'
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        state._sub._callback(OutcomeRequest(target='subject', outcome=0), 'flexbe/command/transition')
        outcome = self._execute(state)
        self.assertEqual(outcome, 'done')
        OperatableStateMachine.autonomy_level = 3
        state.result = None

        # manual transition blocked by lock
        state._sub._callback(String(data='/subject'), 'flexbe/command/lock')
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        state._sub._callback(OutcomeRequest(target='subject', outcome=1), 'flexbe/command/transition')
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        state._sub._callback(String(data='/subject'), 'flexbe/command/unlock')
        outcome = self._execute(state)
        self.assertEqual(outcome, 'error')

        # preempt works on low autonomy
        OperatableStateMachine.autonomy_level = 0
        state.result = 'error'
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        state._sub._callback(Empty(), 'flexbe/command/preempt')
        outcome = self._execute(state)
        self.assertEqual(outcome, PreemptableState._preempted_name)
        PreemptableState.preempt = False
        OperatableStateMachine.autonomy_level = 3
        state.result = None

        # preempt also works when locked
        state._sub._callback(String(data='/subject'), 'flexbe/command/lock')
        outcome = self._execute(state)
        self.assertIsNone(outcome)
        state._sub._callback(Empty(), 'flexbe/command/preempt')
        outcome = self._execute(state)
        self.assertEqual(outcome, PreemptableState._preempted_name)
        PreemptableState.preempt = False
        state._sub._callback(String(data='/subject'), 'flexbe/command/unlock')
        outcome = self._execute(state)
        self.assertIsNone(outcome)



    # Problem with getting locked when sleeping
    # def test_concurrency_container(self):
    #     rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
    #     SubjectStateTest.initialize_ros(self.node)
    #     ConcurrencyContainer.initialize_ros(self.node)
    #     cc = ConcurrencyContainer(outcomes=['done', 'error'],
    #                               conditions=[
    #                                 ('error', [('main', 'error')]),
    #                                 ('error', [('side', 'error')]),
    #                                 ('done', [('main', 'done'), ('side', 'done')])
    #                               ])
    #     with cc:
    #         OperatableStateMachine.add('main', SubjectStateTest(),
    #                                    transitions={'done': 'done', 'error': 'error'},
    #                                    autonomy={'done': 1, 'error': 2})
    #         OperatableStateMachine.add('side', SubjectStateTest(),
    #                                    transitions={'done': 'done', 'error': 'error'},
    #                                    autonomy={'done': 1, 'error': 2})
    #     with OperatableStateMachine(outcomes=['done', 'error']):
    #         OperatableStateMachine.add('cc', cc,
    #                                    transitions={'done': 'done', 'error': 'error'},
    #                                    autonomy={'done': 1, 'error': 2})
    #
    #     class FakeRate(object):
    #
    #         def remaining(self):
    #             return rclpy.Duration(0)
    #
    #         def sleep(self):
    #             pass
    #
    #     # all states are called with their correct rate
    #     cc.execute(None)
    #     print("About to sleep")
    #     thread = threading.Thread(target=rclpy.spin, args=([self.node]), daemon=True)
    #     thread.start()
    #
    #     cc.sleep()
    #     print("Awake again")
    #     cc.execute(None)
    #     self.assertAlmostEqual(cc.sleep_duration, .1, places=2)
    #     cc.sleep()
    #     cc['main'].set_rate(15)
    #     cc['side'].set_rate(10)
    #     cc['main'].count = 0
    #     cc['side'].count = 0
    #     start = self.node.get_clock().now()
    #     cc_count = 0
    #     while self.node.get_clock().now() - start <= 1.:
    #         cc_count += 1
    #         cc.execute(None)
    #         self.assertLessEqual(cc.sleep_duration, .1)
    #         cc.sleep()
    #     self.assertIn(cc['main'].count, [14, 15, 16])
    #     self.assertIn(cc['side'].count, [9, 10, 11])
    #     self.assertLessEqual(cc_count, 27)
    #
    #     # verify ROS properties and disable sleep
    #     cc._enable_ros_control()
    #     self.assertTrue(cc['main']._is_controlled)
    #     self.assertFalse(cc['side']._is_controlled)
    #     cc['main']._rate = FakeRate()
    #     cc['side']._rate = FakeRate()
    #
    #     # return outcome when all return done or any returns error
    #     outcome = cc.execute(None)
    #     self.assertIsNone(outcome)
    #     cc['main'].result = 'error'
    #     outcome = cc.execute(None)
    #     self.assertEqual(outcome, 'error')
    #     cc['main'].result = None
    #     cc['side'].result = 'error'
    #     outcome = cc.execute(None)
    #     self.assertEqual(outcome, 'error')
    #     cc['side'].result = 'done'
    #     outcome = cc.execute(None)
    #     self.assertIsNone(outcome)
    #     cc['main'].result = 'done'
    #     outcome = cc.execute(None)
    #     self.assertEqual(outcome, 'done')
    #     cc['main'].result = None
    #     cc['side'].result = None
    #
    #     # always call on_exit exactly once when returning an outcome
    #     outcome = cc.execute(None)
    #     self.assertIsNone(outcome)
    #     cc['main'].last_events = []
    #     cc['side'].last_events = []
    #     cc['main'].result = 'error'
    #     outcome = cc.execute(None)
    #     self.assertEqual(outcome, 'error')
    #     self.assertListEqual(cc['main'].last_events, ['on_exit'])
    #     self.assertListEqual(cc['side'].last_events, ['on_exit'])
    #
    #     rclpy.shutdown()
    #     thread.join()


    def test_user_data(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        class TestUserdata(EventState):

            def __init__(self, out_content='test_data'):
                super(TestUserdata, self).__init__(outcomes=['done'], input_keys=['data_in'], output_keys=['data_out'])
                self.data = None
                self._out_content = out_content

            def execute(self, userdata):
                self._node.get_logger().warn('\033[0m%s\n%s' % (self.path, str(userdata))) # log for manual inspection
                self.data = userdata.data_in
                userdata.data_out = self._out_content
                return 'done'

        TestUserdata.initialize_ros(self.node)
        OperatableStateMachine.initialize_ros(self.node)
        inner_sm = OperatableStateMachine(outcomes=['done'], input_keys=['sm_in'], output_keys=['sm_out'])
        inner_sm.userdata.own = 'own_data'
        with inner_sm:
            OperatableStateMachine.add('own_state', TestUserdata('inner_data'), transitions={'done': 'outside_state'},
                                       remapping={'data_in': 'own', 'data_out': 'sm_out'})
            OperatableStateMachine.add('outside_state', TestUserdata(), transitions={'done': 'internal_state'},
                                       remapping={'data_in': 'sm_in', 'data_out': 'data_in'})
            OperatableStateMachine.add('internal_state', TestUserdata(), transitions={'done': 'done'},
                                       remapping={})

        sm = OperatableStateMachine(outcomes=['done'])
        sm.userdata.outside = 'outside_data'
        with sm:
            OperatableStateMachine.add('before_state', TestUserdata(), transitions={'done': 'inner_sm'},
                                       remapping={'data_in': 'outside'})
            OperatableStateMachine.add('inner_sm', inner_sm, transitions={'done': 'after_state'},
                                       remapping={'sm_in': 'outside'})
            OperatableStateMachine.add('after_state', TestUserdata('last_data'), transitions={'done': 'modify_state'},
                                       remapping={'data_in': 'sm_out'})
            OperatableStateMachine.add('modify_state', TestUserdata(), transitions={'done': 'final_state'},
                                       remapping={'data_out': 'outside', 'data_in': 'outside'})
            OperatableStateMachine.add('final_state', TestUserdata(), transitions={'done': 'done'},
                                       remapping={'data_in': 'data_out'})

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


if __name__ == '__main__':
    unittest.main()
