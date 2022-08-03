#!/usr/bin/env python
import unittest
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached, ProxyActionClient, ProxyServiceCaller

from std_msgs.msg import String
from std_srvs.srv import Trigger
from flexbe_msgs.action import BehaviorExecution

class TestProxies(unittest.TestCase):
    @classmethod
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.executor = MultiThreadedExecutor(context=self.context)
        self.node = rclpy.create_node('TestProxies', context=self.context)

    @classmethod
    def tearDown(self):
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)

    def test_publish_subscribe(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        ProxyPublisher._initialize(self.node)
        ProxySubscriberCached._initialize(self.node)

        t1 = '/pubsub_1'
        t2 = '/pubsub_2'
        pub = ProxyPublisher({t1: String})
        pub = ProxyPublisher({t2: String})
        sub = ProxySubscriberCached({t1: String}, id=id(self))

        self.assertTrue(pub.is_available(t1))

        self.assertTrue(pub.wait_for_any(t1))
        self.assertFalse(pub.wait_for_any(t2))

        msg1 = String()
        msg1.data = '1'
        msg2 = String()
        msg2.data = '2'

        self.assertTrue(pub.is_available(t1))
        self.assertTrue(pub.is_available(t2))

        sub = ProxySubscriberCached({t2: String}, id=id(self))

        pub.publish(t1, msg1)
        pub.publish(t2, msg2)

        # Make sure messages are sent before checking subscription
        end_time = time.time() + 10
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

        self.assertTrue(sub.has_msg(t1))
        self.assertEqual(sub.get_last_msg(t1).data, '1')
        sub.remove_last_msg(t1)

        self.assertFalse(sub.has_msg(t1))
        self.assertIsNone(sub.get_last_msg(t1))

        self.assertTrue(sub.has_msg(t2))
        self.assertEqual(sub.get_last_msg(t2).data, '2')

    def test_subscribe_buffer(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=5)
        ProxyPublisher._initialize(self.node)
        ProxySubscriberCached._initialize(self.node)

        t1 = '/buffered_1'
        pub = ProxyPublisher({t1: String})
        sub = ProxySubscriberCached({t1: String}, id=id(self))
        sub.enable_buffer(t1)
        self.assertTrue(pub.wait_for_any(t1))

        msg1 = String()
        msg1.data = '1'
        msg2 = String()
        msg2.data = '2'

        pub.publish(t1, msg1)
        pub.publish(t1, msg2)

        # make sure messages can be received
        end_time = time.time() + 10
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

        self.assertTrue(sub.has_msg(t1))
        self.assertTrue(sub.has_buffered(t1))
        self.assertEqual(sub.get_from_buffer(t1).data, '1')

        msg3 = String()
        msg3.data = '3'
        pub.publish(t1, msg3)

        # make sure message can be received
        end_time = time.time() + 10
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

        self.assertEqual(sub.get_from_buffer(t1).data, '2')
        self.assertEqual(sub.get_from_buffer(t1).data, '3')
        self.assertIsNone(sub.get_from_buffer(t1))
        self.assertFalse(sub.has_buffered(t1))

    def test_service_caller(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=5)
        ProxyServiceCaller._initialize(self.node)

        t1 = '/service_1'

        def server_callback(request, response):
            response.success = True
            response.message = "ok"
            return response

        self.node.create_service(Trigger, t1, server_callback)

        srv = ProxyServiceCaller({t1: Trigger})

        srv.call_async(t1, Trigger.Request())
        end_time = time.time() + 10
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

        self.assertTrue(srv.done(t1))

        self.assertIsNotNone(srv.result(t1))
        self.assertTrue(srv.result(t1).success)
        self.assertEqual(srv.result(t1).message, 'ok')

        self.assertFalse(srv.is_available('/not_there'))
        srv = ProxyServiceCaller({'/invalid': Trigger}, wait_duration=.1)
        self.assertFalse(srv.is_available('/invalid'))

    def test_action_client(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=5)
        t1 = '/action_1'
        server = None

        def execute_cb(goal_handle):
            time.sleep(0.1)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return BehaviorExecution.Result()
            else:
                goal_handle.succeed()
                result = BehaviorExecution.Result()
                result.outcome = 'ok'
                return result

        server = ActionServer(self.node, BehaviorExecution, t1, execute_cb)

        ProxyActionClient._initialize(self.node)
        client = ProxyActionClient({t1: BehaviorExecution})
        self.assertFalse(client.has_result(t1))
        client.send_goal(t1, BehaviorExecution.Goal())

        end_time = time.time() + 10
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
            self.assertTrue(client.is_active(t1) or client.has_result(t1))

        self.assertTrue(client.has_result(t1))

        result = client.get_result(t1)
        self.assertEqual(result.outcome, 'ok')

        client.send_goal(t1, BehaviorExecution.Goal())

        # end_time = time.time() + 2
        while not client.has_result(t1):
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
            self.assertTrue(client.is_active(t1) or client.has_result(t1))

        self.assertFalse(client.is_active(t1))

        self.assertFalse(client.is_available('/not_there'))
        client = ProxyActionClient({'/invalid': BehaviorExecution}, wait_duration=.1)
        self.assertFalse(client.is_available('/invalid'))


if __name__ == '__main__':
    unittest.main()
