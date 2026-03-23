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


"""Test the FlexBE proxies."""

import time
import unittest

from action_msgs.msg import GoalStatus

from flexbe_core.core.exceptions import ProxyAvailabilityError
from flexbe_core.proxy import ProxyActionClient, ProxyPublisher, ProxyServiceCaller, ProxySubscriberCached
from flexbe_core.proxy import initialize_proxies, shutdown_proxies

from flexbe_msgs.action import BehaviorExecution

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String

from std_srvs.srv import Trigger


class TestProxies(unittest.TestCase):
    """Test the FlexBE proxies."""

    test = 0
    __EXECUTE_TIMEOUT_SEC = 0.05
    __TIME_SLEEP = 0.01

    def __init__(self, *args, **kwargs):
        """Initialize TestProxies instance."""
        super().__init__(*args, **kwargs)

    def _spin_until(self, predicate, timeout_sec=5.0, message='Timed out waiting for condition'):
        """Spin executor until predicate returns True or timeout expires."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestProxies.__EXECUTE_TIMEOUT_SEC)
            if predicate():
                return
        self.fail(message)

    def _spin_for(self, duration_sec):
        """Spin the executor for a bounded amount of wall-clock time."""
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self.node,
                            executor=self.executor,
                            timeout_sec=min(TestProxies.__EXECUTE_TIMEOUT_SEC, max(0.0, remaining)))

    def setUp(self):
        """Set up the test."""
        TestProxies.test += 1

        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)

        self.executor = MultiThreadedExecutor(context=self.context)
        self.node = rclpy.create_node('proxy_test' + str(self.test), context=self.context)
        self.executor.add_node(self.node)

        self.node.get_logger().info(' set up proxies test %d (%d) ... ' % (self.test, self.context.ok()))
        initialize_proxies(self.node)

    def tearDown(self):
        """Tear down the test."""
        self.node.get_logger().info(' shutting down proxies test %d (%d) ... ' % (self.test, self.context.ok()))
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestProxies.__EXECUTE_TIMEOUT_SEC)

        self.node.get_logger().info('    shutting down proxies in core test %d ... ' % (self.test))
        shutdown_proxies()
        self._spin_for(TestProxies.__EXECUTE_TIMEOUT_SEC)

        self.node.get_logger().info('    destroy node in core test %d ... ' % (self.test))
        self.node.destroy_node()

        self.executor.shutdown()

        # Kill it with fire to make sure not stray published topics are available
        rclpy.shutdown(context=self.context)

    def test_publish_subscribe(self):
        """Test publish and subscribe."""
        self.node.get_logger().info('test_publish_subscribe ...')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestProxies.__EXECUTE_TIMEOUT_SEC)
        ProxyPublisher.initialize(self.node)
        ProxySubscriberCached.initialize(self.node)

        topic1 = '/pubsub_1'
        topic2 = '/pubsub_2'

        self.node.get_logger().info('test_publish_subscribe - define publishers ...')
        pub = ProxyPublisher({topic1: String})
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestProxies.__EXECUTE_TIMEOUT_SEC)
        pub = ProxyPublisher({topic2: String})
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestProxies.__EXECUTE_TIMEOUT_SEC)

        self.node.get_logger().info('  subscribe topic1 only ...')

        sub = ProxySubscriberCached({topic1: String}, inst_id=id(self))

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestProxies.__EXECUTE_TIMEOUT_SEC)
        self.assertTrue(pub.is_available(topic1))

        # cannot call wait given spin_once structure
        # self.assertTrue(pub.wait_for_any(topic1))
        # self.assertFalse(pub.wait_for_any(topic2))
        self._spin_until(lambda: pub.number_of_subscribers(topic1) > 0,
                         timeout_sec=2.0,
                         message='topic1 subscriber did not connect')

        self.assertTrue(pub.number_of_subscribers(topic1) > 0)
        self.assertFalse(pub.number_of_subscribers(topic2) > 0)

        self.node.get_logger().info('  both available ...')
        self.assertTrue(pub.is_available(topic1))
        self.assertTrue(pub.is_available(topic2))

        self.node.get_logger().info('  subscribe topic2 ...')
        sub = ProxySubscriberCached({topic2: String}, inst_id=id(self))
        self._spin_until(lambda: pub.number_of_subscribers(topic1) > 0 and pub.number_of_subscribers(topic2) > 0,
                         timeout_sec=2.0,
                         message='topic subscribers did not connect')

        self.assertTrue(pub.number_of_subscribers(topic1) > 0)
        self.assertTrue(pub.number_of_subscribers(topic2) > 0)

        self.node.get_logger().info('  found both subscribers ...')
        self.assertTrue(pub.is_available(topic1))
        self.assertTrue(pub.is_available(topic2))

        self.node.get_logger().info('  publish two messages ...')
        msg1 = String()
        msg1.data = '1'
        msg2 = String()
        msg2.data = '2'

        pub.publish(topic1, msg1)
        pub.publish(topic2, msg2)

        # Make sure messages are sent before checking subscription
        self.node.get_logger().info('  listen for two messages ...')
        self._spin_until(lambda: sub.has_msg(topic1) and sub.has_msg(topic2),
                         timeout_sec=2.0,
                         message='Did not receive published messages')

        self.assertTrue(sub.has_msg(topic1))
        self.assertEqual(sub.get_last_msg(topic1).data, '1')
        sub.remove_last_msg(topic1)
        self.node.get_logger().info('  received on topic1...')

        self.assertFalse(sub.has_msg(topic1))
        self.assertIsNone(sub.get_last_msg(topic1))

        self.node.get_logger().info('  check for topic2 ...')
        self.assertTrue(sub.has_msg(topic2))
        self.assertEqual(sub.get_last_msg(topic2).data, '2')
        self.node.get_logger().info('test_publish_subscribe - OK!')

    def test_subscribe_buffer(self):
        """Test subscribe buffer."""
        self.node.get_logger().info('test_subscribe_buffer ...')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=2)
        ProxyPublisher.initialize(self.node)
        ProxySubscriberCached.initialize(self.node)

        topic1 = '/buffered_1'
        pub = ProxyPublisher({topic1: String})
        sub = ProxySubscriberCached({topic1: String}, inst_id=id(self))
        sub.enable_buffer(topic1)
        # No wait in this setup -  self.assertTrue(pub.wait_for_any(topic1))
        self._spin_until(lambda: pub.number_of_subscribers(topic1) > 0,
                         timeout_sec=2.0,
                         message='buffered topic subscriber did not connect')

        self.assertTrue(pub.number_of_subscribers(topic1) > 0)

        msg1 = String()
        msg1.data = '1'
        msg2 = String()
        msg2.data = '2'

        pub.publish(topic1, msg1)
        pub.publish(topic1, msg2)

        # make sure messages can be received
        self._spin_until(lambda: sub.has_buffered(topic1)
                         and sub.has_msg(topic1)
                         and sub.get_last_msg(topic1).data == '2',
                         timeout_sec=2.0,
                         message='Buffered messages did not arrive')

        self.assertTrue(sub.has_msg(topic1))
        self.assertTrue(sub.has_buffered(topic1))
        self.assertEqual(sub.get_from_buffer(topic1).data, '1')

        msg3 = String()
        msg3.data = '3'
        pub.publish(topic1, msg3)

        # make sure message can be received
        self._spin_until(lambda: sub.has_buffered(topic1)
                         and sub.peek_at_buffer(topic1) is not None
                         and sub.peek_at_buffer(topic1).data == '2'
                         and sub.has_msg(topic1)
                         and sub.get_last_msg(topic1).data == '3',
                         timeout_sec=2.0,
                         message='Follow-up buffered messages did not arrive')

        self.assertEqual(sub.get_from_buffer(topic1).data, '2')
        self.assertEqual(sub.get_from_buffer(topic1).data, '3')
        self.assertIsNone(sub.get_from_buffer(topic1))
        self.assertFalse(sub.has_buffered(topic1))
        self.node.get_logger().info('test_subscribe_buffer - OK! ')

    def test_service_caller(self):
        """Test service caller."""
        self.node.get_logger().info('test_service_caller ...')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=2)
        ProxyServiceCaller.initialize(self.node)

        topic1 = '/service_1'

        def server_callback(request, response):
            response.success = True
            response.message = 'ok'
            return response

        self.node.create_service(Trigger, topic1, server_callback)

        srv = ProxyServiceCaller({topic1: Trigger})

        srv.call_async(topic1, Trigger.Request())
        self._spin_until(lambda: srv.done(topic1),
                         timeout_sec=3.0,
                         message='Service call did not complete')

        self.assertTrue(srv.done(topic1))

        self.assertIsNotNone(srv.result(topic1))
        self.assertTrue(srv.result(topic1).success)
        self.assertEqual(srv.result(topic1).message, 'ok')

        self.assertFalse(srv.is_available('/not_there'))
        srv = ProxyServiceCaller({'/invalid': Trigger}, wait_duration=.1)
        self.assertFalse(srv.is_available('/invalid'))
        self.node.get_logger().info('test_service_caller  - OK! ')

    def test_service_caller_raises_proxy_availability_error(self):
        """Test unavailable service raises typed proxy availability exception."""
        self.node.get_logger().info('test_service_caller_raises_proxy_availability_error ...')
        ProxyServiceCaller.initialize(self.node)
        topic = '/service_missing'
        srv = ProxyServiceCaller({topic: Trigger}, wait_duration=.01)
        with self.assertRaises(ProxyAvailabilityError):
            srv.call_async(topic, Trigger.Request(), wait_duration=.01)
        self.node.get_logger().info('test_service_caller_raises_proxy_availability_error - OK! ')

    def test_service_caller_sync_request_type_reload(self):
        """Test synchronous service conversion for reloaded request class."""
        self.node.get_logger().info('test_service_caller_sync_request_type_reload ...')
        topic = '/service_reload'

        def server_callback(request, response):
            response.success = True
            response.message = 'ok'
            return response

        self.node.create_service(Trigger, topic, server_callback)
        ProxyServiceCaller.initialize(self.node)
        srv = ProxyServiceCaller({topic: Trigger}, wait_duration=1.0)
        self._spin_until(lambda: srv.is_available(topic, wait_duration=0.01),
                         timeout_sec=3.0,
                         message='Service did not become available for reload request test')

        base_req = Trigger.Request()
        reloaded_request_name = Trigger.Request.__name__
        ReloadedRequest = type(reloaded_request_name, (), {'__slots__': list(base_req.__slots__)})
        request = ReloadedRequest()
        for attr in base_req.__slots__:
            setattr(request, attr, getattr(base_req, attr))

        srv.call_async(topic, request, wait_duration=1.0)
        self._spin_until(lambda: srv.done(topic),
                         timeout_sec=3.0,
                         message='Service result did not complete for reload request test')
        response = srv.result(topic)
        self.assertIsNotNone(response)
        self.assertTrue(response.success)
        self.assertEqual(response.message, 'ok')
        self.node.get_logger().info('test_service_caller_sync_request_type_reload - OK! ')

    def test_action_client(self):
        """Test action client."""
        self.node.get_logger().info('test_action_client ...')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=2)
        topic1 = '/action_1'

        def execute_cb(goal_handle):
            time.sleep(max(TestProxies.__TIME_SLEEP, 0.05))
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return BehaviorExecution.Result()

            goal_handle.succeed()
            result = BehaviorExecution.Result()
            result.outcome = 'ok'
            return result

        server = ActionServer(self.node, BehaviorExecution, topic1, execute_cb)

        ProxyActionClient.initialize(self.node)
        client = ProxyActionClient({topic1: BehaviorExecution}, wait_duration=1.0)
        self.assertFalse(client.has_result(topic1))
        status = client.get_status(topic1)
        self.node.get_logger().info(f'validate action client - check status before sending = {status} ')

        client.send_goal(topic1, BehaviorExecution.Goal(), wait_duration=1.0)
        status = client.get_status(topic1)
        self.node.get_logger().info(f'validate action client - check status after goal sent = {status} ')

        self._spin_until(lambda: client.has_result(topic1)
                         and client.get_result(topic1).outcome == 'ok'
                         and not client.is_active(topic1),
                         timeout_sec=3.0,
                         message='First action result did not arrive')

        self.assertTrue(client.has_result(topic1))

        status = client.get_status(topic1)
        self.node.get_logger().info(f'   check status = {status} ')

        self.node.get_logger().info('validate action client result 1 ... ')
        result = client.get_result(topic1)
        self.assertEqual(result.outcome, 'ok')

        status = client.get_status(topic1)
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        self.node.get_logger().info('validate action client succeeded - OK! ')

        status = client.get_status(topic1)
        self.node.get_logger().info(f'   check status before send 2 = {status} ')
        client.send_goal(topic1, BehaviorExecution.Goal(), wait_duration=1.0)
        status = client.get_status(topic1)
        self.node.get_logger().info(f'   check status after sending goal 2 = {status} ')

        # end_time = time.time() + 2
        self._spin_until(lambda: client.has_result(topic1)
                         and client.get_result(topic1).outcome == 'ok'
                         and not client.is_active(topic1),
                         timeout_sec=3.0,
                         message='Second action result did not arrive')

        self.assertFalse(client.is_active(topic1))

        self.node.get_logger().info('validate action client result 2 ... ')
        result = client.get_result(topic1)
        self.assertEqual(result.outcome, 'ok')

        status = client.get_status(topic1)
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        self.node.get_logger().info('validate action client succeeded 2 - OK! ')

        self.assertTrue(client.has_result(topic1))
        client.remove_result(topic1)
        self.assertIsNone(client._result.get(topic1))
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        #  -- we are now preserving status until cleared on next goal
        # self.assertIsNone(client._result_status.get(topic1))
        self.assertFalse(client.has_result(topic1))
        self.node.get_logger().info('validated remove_result! ')

        self.assertFalse(client.is_available('/not_there'))
        client = ProxyActionClient({'/invalid': BehaviorExecution}, wait_duration=.1)
        self.assertFalse(client.is_available('/invalid'))
        self.node.get_logger().info('test_action_client - OK! ')
        del server  # Through with instance, and explicitly calling del() to avoid unused warning

    def test_action_client_raises_proxy_availability_error(self):
        """Test unavailable action server raises typed proxy availability exception."""
        self.node.get_logger().info('test_action_client_raises_proxy_availability_error ...')
        ProxyActionClient.initialize(self.node)
        topic = '/action_missing'
        client = ProxyActionClient({topic: BehaviorExecution}, wait_duration=.01)
        with self.assertRaises(ProxyAvailabilityError):
            client.send_goal(topic, BehaviorExecution.Goal(), wait_duration=.01)
        self.node.get_logger().info('test_action_client_raises_proxy_availability_error - OK! ')

    def test_action_client_goal_type_reload(self):
        """Test send_goal conversion for a reloaded action goal class."""
        self.node.get_logger().info('test_action_client_goal_type_reload ...')
        topic = '/action_reload'

        def execute_cb(goal_handle):
            goal_handle.succeed()
            result = BehaviorExecution.Result()
            result.outcome = 'ok'
            return result

        server = ActionServer(self.node, BehaviorExecution, topic, execute_cb)
        ProxyActionClient.initialize(self.node)
        client = ProxyActionClient({topic: BehaviorExecution}, wait_duration=1.0)
        self._spin_until(lambda: client.is_available(topic), timeout_sec=3.0,
                         message='Action server did not become available for reload goal test')

        base_goal = BehaviorExecution.Goal()
        reloaded_goal_name = BehaviorExecution.Goal.__name__
        ReloadedGoal = type(reloaded_goal_name, (), {'__slots__': list(base_goal.__slots__)})
        goal = ReloadedGoal()
        for attr in base_goal.__slots__:
            setattr(goal, attr, getattr(base_goal, attr))

        client.send_goal(topic, goal, wait_duration=1.0)
        self._spin_until(lambda: client.has_result(topic), timeout_sec=5.0,
                         message='Action result not received for reload goal test')

        result = client.get_result(topic)
        self.assertEqual(result.outcome, 'ok')
        self.assertEqual(client.get_status(topic), GoalStatus.STATUS_SUCCEEDED)
        self.node.get_logger().info('test_action_client_goal_type_reload - OK! ')
        del server


if __name__ == '__main__':
    unittest.main()
