#!/usr/bin/env python

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


"""Test for onboard behaviors."""

import sys
import multiprocessing
import os
import unittest
import zlib
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor

from flexbe_onboard import FlexbeOnboard
from flexbe_core.core.topics import Topics
from flexbe_core.proxy import ProxySubscriberCached

from flexbe_msgs.msg import BehaviorSelection, BEStatus, BehaviorLog, BehaviorModification


class TestOnboard(unittest.TestCase):
    """Test for onboard behaviors."""

    @classmethod
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        num_threads = max(2, multiprocessing.cpu_count() - 1)
        self.executor = MultiThreadedExecutor(num_threads=num_threads, context=self.context)
        self.node = rclpy.create_node('TestOnboard', context=self.context)
        self.executor.add_node(self.node)

        ProxySubscriberCached.initialize(self.node)

        self.sub = ProxySubscriberCached({Topics._ONBOARD_STATUS_TOPIC: BEStatus,
                                          Topics._BEHAVIOR_LOGGING_TOPIC: BehaviorLog},
                                         inst_id=id(self))
        # make sure that behaviors can be imported
        data_folder = os.path.dirname(os.path.realpath(__file__))
        sys.path.insert(0, data_folder)

        # run onboard and add custom test behaviors to onboard lib
        rclpy.init()
        self.onboard = FlexbeOnboard()
        self.executor.add_node(self.onboard)

        self.lib = self.onboard._behavior_lib
        self.lib._add_behavior_manifests(data_folder)

    @classmethod
    def tearDown(self):
        self.node.destroy_node()
        self.onboard.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)

    def assertStatus(self, expected, timeout, message=""):
        """Assert that the expected onboard status is received before the timeout."""
        self.executor.spin_once(timeout_sec=0.01)
        for i in range(int(timeout * 100)):

            # Spin both nodes as needed
            self.executor.spin_once(timeout_sec=0.01)
            time.sleep(0.1)
            if self.sub.has_msg(Topics._ONBOARD_STATUS_TOPIC):
                break
        else:
            raise AssertionError('Did not receive a status as required.')
        msg = self.sub.get_last_msg(Topics._ONBOARD_STATUS_TOPIC)
        self.sub.remove_last_msg(Topics._ONBOARD_STATUS_TOPIC)
        self.node.get_logger().info(f"assertStatus: msg= {str(msg)} expected={expected} - {message}?")
        self.assertEqual(msg.code, expected, msg=message)
        return msg

    def clear_extra_heartbeat_ready_messages(self):
        while self.sub.has_msg(Topics._ONBOARD_STATUS_TOPIC):
            msg = self.sub.get_last_msg(Topics._ONBOARD_STATUS_TOPIC)
            if msg.code == BEStatus.READY:
                self.node.get_logger().info(f"clearing READY msg={str(msg)}")
                self.sub.remove_last_msg(Topics._ONBOARD_STATUS_TOPIC)  # Clear any heartbeat start up messages
            else:
                break
            self.executor.spin_once(timeout_sec=0.01)

    def test_onboard_behaviors(self):
        self.executor.spin_once(timeout_sec=1)

        behavior_pub = self.node.create_publisher(BehaviorSelection, Topics._START_BEHAVIOR_TOPIC, 1)
        # wait for publisher2
        end_time = time.time() + 3.0
        while time.time() < end_time:
            self.executor.spin_once(timeout_sec=0.1)

        # wait for the initial status message
        self.assertStatus(BEStatus.READY, 1, "BE is ready")

        # send simple behavior request without checksum
        be_key, _ = self.lib.find_behavior("Log Behavior Test")
        request = BehaviorSelection()
        request.behavior_key = be_key
        request.autonomy_level = 255

        self.clear_extra_heartbeat_ready_messages()

        self.node.get_logger().info("Publish request ...")
        behavior_pub.publish(request)
        self.executor.spin_once(timeout_sec=0.1)

        self.node.get_logger().info("Check for expected error ...")
        self.assertStatus(BEStatus.ERROR, 2, "Error - checksum test")

        # send valid simple behavior request
        with open(self.lib.get_sourcecode_filepath(be_key)) as f:
            request.behavior_id = zlib.adler32(f.read().encode()) & 0x7fffffff
        self.sub.enable_buffer(Topics._BEHAVIOR_LOGGING_TOPIC)

        self.clear_extra_heartbeat_ready_messages()

        self.node.get_logger().info("Publish with checksum ...")
        behavior_pub.publish(request)
        self.executor.spin_once(timeout_sec=0.1)

        self.assertStatus(BEStatus.STARTED, 1, "Started simple log behavior")
        self.assertStatus(BEStatus.FINISHED, 3, "Finished simple log behavior")
        behavior_logs = []
        while self.sub.has_buffered(Topics._BEHAVIOR_LOGGING_TOPIC):
            behavior_logs.append(self.sub.get_from_buffer(Topics._BEHAVIOR_LOGGING_TOPIC).text)
            self.executor.spin_once(timeout_sec=0.1)
        self.assertIn('Test data', behavior_logs)

        # send valid complex behavior request
        self.node.get_logger().info("Request to find (INVALID) complex behavior ...")
        be_key, _ = self.lib.find_behavior("Complex Behavior Test")
        request = BehaviorSelection()
        request.behavior_key = be_key
        request.autonomy_level = 255
        request.arg_keys = ['param']
        request.arg_values = ['value_2']
        request.input_keys = ['data']
        request.input_values = ['2']

        with open(self.lib.get_sourcecode_filepath(be_key)) as f:
            content = f.read()
        self.node.get_logger().info("Request behavior modification of (INVALID) complex behavior ...")
        modifications = [('INVALID', 'core'), ('raise ValueError("TODO: Remove!")', '')]
        for replace, by in modifications:
            index = content.index(replace)
            request.modifications.append(BehaviorModification(index_begin=index, index_end=index + len(replace), new_content=by))
        for replace, by in modifications:
            content = content.replace(replace, by)

        self.node.get_logger().info("Modified modified behavior ...")
        self.node.get_logger().info(content)
        self.node.get_logger().info(30 * "=" + "\n\n")
        request.behavior_id = zlib.adler32(content.encode()) & 0x7fffffff

        self.clear_extra_heartbeat_ready_messages()

        self.node.get_logger().info("Publish modified behavior ...")
        behavior_pub.publish(request)
        self.executor.spin_once(timeout_sec=0.1)
        self.assertStatus(BEStatus.STARTED, 1, "Started modified")
        result = self.assertStatus(BEStatus.FINISHED, 3, "Finished modified")
        self.assertEqual(result.args[0], 'finished')
        behavior_logs = []

        # Wait for published message
        end_time = time.time() + 1
        while time.time() < end_time:
            self.executor.spin_once(timeout_sec=0.05)

        while self.sub.has_buffered(Topics._BEHAVIOR_LOGGING_TOPIC):
            behavior_logs.append(self.sub.get_from_buffer(Topics._BEHAVIOR_LOGGING_TOPIC).text)
        self.assertIn('value_2', behavior_logs)

        self.clear_extra_heartbeat_ready_messages()

        # send the same behavior with different parameters
        self.node.get_logger().info("Republish modified behavior ...")
        request.arg_keys = ['param', 'invalid']
        request.arg_values = ['value_1', 'should be ignored']
        request.input_keys = []
        request.input_values = []
        behavior_pub.publish(request)
        self.executor.spin_once(timeout_sec=0.1)
        self.assertStatus(BEStatus.STARTED, 1, "Started modified parameters")
        result = self.assertStatus(BEStatus.FINISHED, 3, "Finished modified parameters")
        self.assertEqual(result.args[0], 'failed')
        behavior_logs = []

        # Wait for published message
        end_time = time.time() + 1
        while time.time() < end_time:
            self.executor.spin_once(timeout_sec=0.1)

        while self.sub.has_buffered(Topics._BEHAVIOR_LOGGING_TOPIC):
            behavior_logs.append(self.sub.get_from_buffer(Topics._BEHAVIOR_LOGGING_TOPIC).text)
            self.executor.spin_once(timeout_sec=0.1)
        self.assertIn('value_1', behavior_logs)
        self.node.get_logger().info("Done onboard testing!")
        self.executor.spin_once(timeout_sec=0.1)


if __name__ == '__main__':
    unittest.main()
