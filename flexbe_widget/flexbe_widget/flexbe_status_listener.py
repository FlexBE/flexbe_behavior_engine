#!/usr/bin/env python3

# Copyright 2024 Christopher Newport University
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
#    * Neither the name of the Christopher Newport University nor the names of its
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

from flexbe_core.core import StateMap, Topics

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


class FlexbeStatusListener(Node):
    """Simple listener to echo FlexBE state machine information."""

    def __init__(self):
        super().__init__('flexbe_status_listener')
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._state_map = None
        self._state_map_sub = self.create_subscription(
            Topics._topic_types[Topics._STATE_MAP_OCS_TOPIC],
            Topics._STATE_MAP_OCS_TOPIC,
            self._state_map_callback,
            latching_qos)
        self._state_map_sub  # prevent unused variable warning

        self._outcome_sub = self.create_subscription(
            Topics._topic_types[Topics._OUTCOME_TOPIC],
            Topics._OUTCOME_TOPIC,
            self._outcome_callback,
            10)
        self._outcome_sub  # prevent unused variable warning

        self._heartbeat_sub = self.create_subscription(
            Topics._topic_types[Topics._ONBOARD_HEARTBEAT_TOPIC],
            Topics._ONBOARD_HEARTBEAT_TOPIC,
            self._heartbeat_callback,
            10)
        self._heartbeat_sub  # prevent unused variable warning

        self._sync_sub = self.create_subscription(
            Topics._topic_types[Topics._MIRROR_SYNC_TOPIC],
            Topics._MIRROR_SYNC_TOPIC,
            self._sync_callback,
            10)
        self._sync_sub  # prevent unused variable warning

    def _heartbeat_callback(self, msg):
        if self._state_map is None:
            self.get_logger().info(f'Onboard heartbeat {msg}')
            return

        states = []
        for checksum in msg.current_state_checksums:
            state_id, _ = StateMap.unhash(checksum)
            states.append(self._state_map.get(state_id, 'unknown'))

        self.get_logger().info(f'Onboard heartbeat {msg.behavior_id:10d} : {states}')

    def _sync_callback(self, msg):
        if self._state_map is None:
            self.get_logger().info(f'Synchronize mirror {msg}')
            return

        states = []
        for checksum in msg.current_state_checksums:
            state_id, _ = StateMap.unhash(checksum)
            states.append(self._state_map.get(state_id, 'unknown'))

        self.get_logger().info(f'Synchronize mirror {msg.behavior_id:10d} : {states}')

    def _outcome_callback(self, msg):
        if self._state_map is None:
            self.get_logger().info(f'Outcome msg hash value {msg.data:11d}')
            return

        state_id, outcome = StateMap.unhash(msg.data)
        path = f"'{self._state_map.get(state_id, 'unknown')}'"
        self.get_logger().info(f'Outcome {outcome:2d} from {state_id:11d} {path:60s}')

    def _state_map_callback(self, msg):
        self.get_logger().info(f'New state map received for {msg.behavior_id}')

        state_map = {}
        for id, path in zip(msg.state_ids, msg.state_paths):
            self.get_logger().info(f"  adding {id:11d} at '{path}'")
            state_map[id] = path

        self._state_map = state_map


def flexbe_status_listener_main(args=None):
    """Run the status listener to echo state info given state id in messages."""
    print('Suggest: export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{time}] {message}" in terminal')
    print(' Run this before starting behavior to get full state path information from state map.')
    rclpy.init(args=args)

    outcomes = FlexbeStatusListener()

    rclpy.spin(outcomes)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    outcomes.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    flexbe_status_listener_main()
