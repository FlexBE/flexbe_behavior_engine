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


"""Test the FlexBE proxies."""

import time
import unittest

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from std_srvs.srv import Trigger
from flexbe_msgs.action import BehaviorExecution

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached, ProxyActionClient, ProxyServiceCaller
from flexbe_core.proxy import initialize_proxies, shutdown_proxies


class TestProxies(unittest.TestCase):
    """Test the FlexBE proxies."""

    test = 0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def setUp(self):
        TestProxies.test += 1

        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)

        self.executor = MultiThreadedExecutor(context=self.context)
        self.node = rclpy.create_node("proxy_test" + str(self.test), context=self.context)
        self.executor.add_node(self.node)

        self.node.get_logger().info(" set up proxies test %d (%d) ... " % (self.test, self.context.ok()))
        initialize_proxies(self.node)

    def tearDown(self):
        self.node.get_logger().info(" shutting down proxies test %d (%d) ... " % (self.test, self.context.ok()))
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
        time.sleep(0.5)

    def test_publish_subscribe(self):
        self.node.get_logger().info("test_publish_subscribe ...")

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        ProxyPublisher.initialize(self.node)
        ProxySubscriberCached.initialize(self.node)

        topic1 = '/pubsub_1'
        topic2 = '/pubsub_2'

        self.node.get_logger().info("test_publish_subscribe - define publishers ...")
        pub = ProxyPublisher({topic1: String})
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
        pub = ProxyPublisher({topic2: String})
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

        self.node.get_logger().info("  subscribe topic1 only ...")

        sub = ProxySubscriberCached({topic1: String}, inst_id=id(self))

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
        self.assertTrue(pub.is_available(topic1))

        # cannot call wait given spin_once structure
        # self.assertTrue(pub.wait_for_any(topic1))
        # self.assertFalse(pub.wait_for_any(topic2))
        for _ in range(50):
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.01)
            time.sleep(0.04)

        self.assertTrue(pub.number_of_subscribers(topic1) > 0)
        self.assertFalse(pub.number_of_subscribers(topic2) > 0)

        self.node.get_logger().info("  both available ...")
        self.assertTrue(pub.is_available(topic1))
        self.assertTrue(pub.is_available(topic2))

        self.node.get_logger().info("  subscribe topic2 ...")
        sub = ProxySubscriberCached({topic2: String}, inst_id=id(self))
        for _ in range(50):
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.01)
            time.sleep(0.05)

        self.assertTrue(pub.number_of_subscribers(topic1) > 0)
        self.assertTrue(pub.number_of_subscribers(topic2) > 0)

        self.node.get_logger().info("  found both subscribers ...")
        self.assertTrue(pub.is_available(topic1))
        self.assertTrue(pub.is_available(topic2))

        self.node.get_logger().info("  publish two messages ...")
        msg1 = String()
        msg1.data = '1'
        msg2 = String()
        msg2.data = '2'

        pub.publish(topic1, msg1)
        pub.publish(topic2, msg2)

        # Make sure messages are sent before checking subscription
        self.node.get_logger().info("  listen for two messages ...")
        end_time = time.time() + 5
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

        self.assertTrue(sub.has_msg(topic1))
        self.assertEqual(sub.get_last_msg(topic1).data, '1')
        sub.remove_last_msg(topic1)
        self.node.get_logger().info("  received on topic1...")

        self.assertFalse(sub.has_msg(topic1))
        self.assertIsNone(sub.get_last_msg(topic1))

        self.node.get_logger().info("  check for topic2 ...")
        self.assertTrue(sub.has_msg(topic2))
        self.assertEqual(sub.get_last_msg(topic2).data, '2')
        self.node.get_logger().info("test_publish_subscribe - OK!")

    def test_subscribe_buffer(self):
        self.node.get_logger().info("test_subscribe_buffer ...")

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=2)
        ProxyPublisher.initialize(self.node)
        ProxySubscriberCached.initialize(self.node)

        topic1 = '/buffered_1'
        pub = ProxyPublisher({topic1: String})
        sub = ProxySubscriberCached({topic1: String}, inst_id=id(self))
        sub.enable_buffer(topic1)
        # No wait in this setup -  self.assertTrue(pub.wait_for_any(topic1))
        for _ in range(10):
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.01)
            time.sleep(0.05)

        self.assertTrue(pub.number_of_subscribers(topic1) > 0)

        msg1 = String()
        msg1.data = '1'
        msg2 = String()
        msg2.data = '2'

        pub.publish(topic1, msg1)
        pub.publish(topic1, msg2)

        # make sure messages can be received
        end_time = time.time() + 3
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

        self.assertTrue(sub.has_msg(topic1))
        self.assertTrue(sub.has_buffered(topic1))
        self.assertEqual(sub.get_from_buffer(topic1).data, '1')

        msg3 = String()
        msg3.data = '3'
        pub.publish(topic1, msg3)

        # make sure message can be received
        end_time = time.time() + 3
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

        self.assertEqual(sub.get_from_buffer(topic1).data, '2')
        self.assertEqual(sub.get_from_buffer(topic1).data, '3')
        self.assertIsNone(sub.get_from_buffer(topic1))
        self.assertFalse(sub.has_buffered(topic1))
        self.node.get_logger().info("test_subscribe_buffer - OK! ")

    def test_service_caller(self):
        self.node.get_logger().info("test_service_caller ...")

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=2)
        ProxyServiceCaller.initialize(self.node)

        topic1 = '/service_1'

        def server_callback(request, response):
            response.success = True
            response.message = "ok"
            return response

        self.node.create_service(Trigger, topic1, server_callback)

        srv = ProxyServiceCaller({topic1: Trigger})

        srv.call_async(topic1, Trigger.Request())
        end_time = time.time() + 10
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

        self.assertTrue(srv.done(topic1))

        self.assertIsNotNone(srv.result(topic1))
        self.assertTrue(srv.result(topic1).success)
        self.assertEqual(srv.result(topic1).message, 'ok')

        self.assertFalse(srv.is_available('/not_there'))
        srv = ProxyServiceCaller({'/invalid': Trigger}, wait_duration=.1)
        self.assertFalse(srv.is_available('/invalid'))
        self.node.get_logger().info("test_service_caller  - OK! ")

    def test_action_client(self):
        self.node.get_logger().info("test_action_client ...")

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=2)
        topic1 = '/action_1'

        def execute_cb(goal_handle):
            time.sleep(0.1)
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
        client.send_goal(topic1, BehaviorExecution.Goal(), wait_duration=1.0)

        end_time = time.time() + 10
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
            self.assertTrue(client.is_active(topic1) or client.has_result(topic1))

        self.assertTrue(client.has_result(topic1))

        result = client.get_result(topic1)
        self.assertEqual(result.outcome, 'ok')

        client.send_goal(topic1, BehaviorExecution.Goal(), wait_duration=1.0)

        # end_time = time.time() + 2
        while not client.has_result(topic1):
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
            self.assertTrue(client.is_active(topic1) or client.has_result(topic1))

        self.assertFalse(client.is_active(topic1))

        self.assertFalse(client.is_available('/not_there'))
        client = ProxyActionClient({'/invalid': BehaviorExecution}, wait_duration=.1)
        self.assertFalse(client.is_available('/invalid'))
        self.node.get_logger().info("test_action_client - OK! ")
        del server  # Through with instance, and explicitly calling del() to avoid ununsed warning


if __name__ == '__main__':
    unittest.main()
