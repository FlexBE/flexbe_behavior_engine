#!/usr/bin/env python
import sys
import os
import unittest
import zlib
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from flexbe_onboard import FlexbeOnboard
from flexbe_core.proxy import ProxySubscriberCached

from flexbe_msgs.msg import BehaviorSelection, BEStatus, BehaviorLog, BehaviorModification


class TestOnboard(unittest.TestCase):
    @classmethod
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.executor = MultiThreadedExecutor(context=self.context)
        self.node = rclpy.create_node('TestOnboard', context=self.context)

        ProxySubscriberCached._initialize(self.node)

        self.sub = ProxySubscriberCached({'flexbe/status': BEStatus,
                                          'flexbe/log': BehaviorLog
                                         }, id=id(self))
        self.rate = self.node.create_rate(100, self.node.get_clock())
        # make sure that behaviors can be imported
        data_folder = os.path.dirname(os.path.realpath(__file__))
        sys.path.insert(0, data_folder)
        # run onboard and add custom test behaviors to onboard lib
        self.onboard = FlexbeOnboard(self.node)
        self.lib = self.onboard._behavior_lib
        self.lib._add_behavior_manifests(data_folder)

    @classmethod
    def tearDown(self):
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)

    def assertStatus(self, expected, timeout):
        """ Assert that the expected onboard status is received before the timeout. """
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)
        for i in range(int(timeout*100)):
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
            # self.rate.sleep()
            time.sleep(0.1)
            if self.sub.has_msg('flexbe/status'):
                break
        else:
            raise AssertionError('Did not receive a status as required.')
        msg = self.sub.get_last_msg('flexbe/status')
        self.sub.remove_last_msg('flexbe/status')
        self.assertEqual(msg.code, expected)
        return msg

    def test_onboard_behaviors(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=1)

        behavior_pub = self.node.create_publisher(BehaviorSelection, 'flexbe/start_behavior', 1)
        # wait for publisher
        end_time = time.time() + 10
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

        # wait for the initial status message
        self.assertStatus(BEStatus.READY, 1)

        # send simple behavior request without checksum
        be_id, _ = self.lib.find_behavior("Test Behavior Log")
        request = BehaviorSelection()
        request.behavior_id = be_id
        request.autonomy_level = 255
        behavior_pub.publish(request)
        self.assertStatus(BEStatus.ERROR, 2)

        # send valid simple behavior request
        with open(self.lib.get_sourcecode_filepath(be_id)) as f:
            request.behavior_checksum = zlib.adler32(f.read().encode()) & 0x7fffffff
        self.sub.enable_buffer('flexbe/log')
        behavior_pub.publish(request)

        self.assertStatus(BEStatus.STARTED, 1)
        self.assertStatus(BEStatus.FINISHED, 3)
        behavior_logs = []
        while self.sub.has_buffered('flexbe/log'):
            behavior_logs.append(self.sub.get_from_buffer('flexbe/log').text)
        self.assertIn('Test data', behavior_logs)

        # send valid complex behavior request
        be_id, _ = self.lib.find_behavior("Test Behavior Complex")
        request = BehaviorSelection()
        request.behavior_id = be_id
        request.autonomy_level = 255
        request.arg_keys = ['param']
        request.arg_values = ['value_2']
        request.input_keys = ['data']
        request.input_values = ['2']

        with open(self.lib.get_sourcecode_filepath(be_id)) as f:
            content = f.read()
        modifications = [('flexbe_INVALID', 'flexbe_core'), ('raise ValueError("TODO: Remove!")', '')]
        for replace, by in modifications:
            index = content.index(replace)
            request.modifications.append(BehaviorModification(index_begin=index, index_end=index + len(replace), new_content=by))
        for replace, by in modifications:
            content = content.replace(replace, by)
        request.behavior_checksum = zlib.adler32(content.encode()) & 0x7fffffff
        behavior_pub.publish(request)
        self.assertStatus(BEStatus.STARTED, 1)
        result = self.assertStatus(BEStatus.FINISHED, 3)
        self.assertEqual(result.args[0], 'finished')
        behavior_logs = []

        # Wait for published message
        end_time = time.time() + 1
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

        while self.sub.has_buffered('flexbe/log'):
            behavior_logs.append(self.sub.get_from_buffer('flexbe/log').text)
        self.assertIn('value_2', behavior_logs)

        # send the same behavior with different parameters
        request.arg_keys = ['param', 'invalid']
        request.arg_values = ['value_1', 'should be ignored']
        request.input_keys = []
        request.input_values = []
        behavior_pub.publish(request)
        self.assertStatus(BEStatus.STARTED, 1)
        result = self.assertStatus(BEStatus.FINISHED, 3)
        self.assertEqual(result.args[0], 'failed')
        behavior_logs = []

        # Wait for published message
        end_time = time.time() + 1
        while time.time() < end_time:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

        while self.sub.has_buffered('flexbe/log'):
            behavior_logs.append(self.sub.get_from_buffer('flexbe/log').text)
        self.assertIn('value_1', behavior_logs)


if __name__ == '__main__':
    unittest.main()
