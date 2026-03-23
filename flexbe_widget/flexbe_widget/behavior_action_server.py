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


"""Behavior action server."""

import difflib
import os
import time
import zlib

from flexbe_core import BehaviorLibrary
from flexbe_core.core.topics import Topics

from flexbe_msgs.action import BehaviorExecution
from flexbe_msgs.msg import BEStatus, BehaviorModification, BehaviorSelection

import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from rosidl_runtime_py import get_interface_path

from std_msgs.msg import Empty, Int32

import yaml


class BehaviorActionServer:
    """Behavior action server."""

    def __init__(self, node):
        self._node = node
        self._behavior_started = False
        self._preempt_requested = False
        self._current_goal = None
        self._requested_behavior_id = None

        self._current_state = None
        self._active_behavior_id = None

        self.topic_group = MutuallyExclusiveCallbackGroup()
        self.action_group = MutuallyExclusiveCallbackGroup()
        status_qos = QoSProfile(depth=20, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self._pub = self._node.create_publisher(BehaviorSelection, Topics._START_BEHAVIOR_TOPIC, 100)
        self._preempt_pub = self._node.create_publisher(Empty, Topics._CMD_PREEMPT_TOPIC, 100)

        self._status_pub = self._node.create_subscription(BEStatus, Topics._ONBOARD_STATUS_TOPIC, self._status_cb,
                                                          qos_profile=status_qos, callback_group=self.topic_group)
        self._state_pub = self._node.create_subscription(Int32, Topics._BEHAVIOR_UPDATE_TOPIC, self._state_cb,
                                                         100, callback_group=self.topic_group)

        self._as = ActionServer(self._node, BehaviorExecution,
                                Topics._EXECUTE_BEHAVIOR_ACTION,
                                goal_callback=self._goal_request_cb,
                                handle_accepted_callback=self._goal_cb,
                                cancel_callback=self._preempt_cb,
                                execute_callback=self._execute_cb,
                                callback_group=self.action_group)

        self._behavior_lib = BehaviorLibrary(node)

        self._node.get_logger().info('%d behaviors available, ready for start request.' % self._behavior_lib.count_behaviors())
        self.running = False
        self.outcome = ''

    def _goal_request_cb(self, _goal_request):
        """Accept or reject goals before the action server creates a goal handle."""
        if self.running or (self._current_goal is not None and self._current_goal.is_active):
            self._node.get_logger().warning('Reject goal: behavior execution is already active.')
            return GoalResponse.REJECT

        if self._preempt_requested:
            self._node.get_logger().warning('Reject goal: a preempt request is still pending.')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _goal_cb(self, goal_handle: ServerGoalHandle):
        self._current_goal = goal_handle
        goal = goal_handle.request
        try:
            if self._preempt_requested:
                goal_handle.canceled()
                self.clean_me()
                return

            self._node.get_logger().info('Received a new request to start behavior: %s' % goal.behavior_name)
            be_key, behavior = self._behavior_lib.find_behavior(goal.behavior_name)
            if be_key is None:
                self._node.get_logger().error('Deny goal: Did not find behavior with requested name %s' % goal.behavior_name)
                self._current_goal.abort()
                return

            be_selection = BehaviorSelection()
            be_selection.behavior_key = be_key
            be_selection.autonomy_level = 255
            if len(goal.arg_keys) != len(goal.arg_values):
                self._node.get_logger().error('Deny goal: arg_keys and arg_values length mismatch '
                                              f'({len(goal.arg_keys)} != {len(goal.arg_values)})')
                self._current_goal.abort()
                return
            if len(goal.input_keys) != len(goal.input_values):
                self._node.get_logger().error('Deny goal: input_keys and input_values length mismatch '
                                              f'({len(goal.input_keys)} != {len(goal.input_values)})')
                self._current_goal.abort()
                return
            try:
                for k, v in zip(goal.arg_keys, goal.arg_values):
                    if v.startswith('file://'):
                        v = v.replace('file://', '', 1)
                        path = v.split(':')[0]
                        if len(v.split(':')) > 1:
                            ns = v.split(':')[1]
                        else:
                            ns = ''
                        if path.startswith('~') or path.startswith('/'):
                            filepath = os.path.expanduser(path)
                        else:
                            filepath = os.path.join(get_interface_path(path.split('/')[0]), '/'.join(path.split('/')[1:]))
                        with open(filepath, 'r') as f:
                            content = f.read()
                        if ns != '':
                            content = yaml.safe_load(content)
                            if ns in content:
                                content = content[ns]
                            content = yaml.safe_dump(content, sort_keys=False)
                        be_selection.arg_keys.append(k)
                        be_selection.arg_values.append(content)
                    else:
                        be_selection.arg_keys.append(k)
                        be_selection.arg_values.append(v)
            except Exception as e:  # noqa: B902
                self._node.get_logger().warning('Failed to parse and substitute behavior arguments,'
                                                ' will use direct input.\n%s' % str(e))
                be_selection.arg_keys = goal.arg_keys
                be_selection.arg_values = goal.arg_values
            be_selection.input_keys = goal.input_keys
            be_selection.input_values = goal.input_values

            # check for local modifications of the behavior to send them to the onboard behavior
            be_filepath_new = self._behavior_lib.get_sourcecode_filepath(be_key)
            with open(be_filepath_new, 'r') as f:
                be_content_new = f.read()

            be_filepath_old = self._behavior_lib.get_sourcecode_filepath(be_key, add_tmp=True)
            if not os.path.isfile(be_filepath_old):
                be_selection.behavior_id = zlib.adler32(be_content_new.encode()) & 0x7fffffff
            else:
                with open(be_filepath_old, 'r') as f:
                    be_content_old = f.read()

                sqm = difflib.SequenceMatcher(a=be_content_old, b=be_content_new)
                diffs = [x[1] for x in sqm.get_grouped_opcodes(0)]
                for opcode, a0, a1, b0, b1 in diffs:  # pylint: disable=W0612
                    content = be_content_new[b0:b1]
                    be_selection.modifications.append(BehaviorModification(index_begin=a0,
                                                                           index_end=a1,
                                                                           new_content=content))

                be_selection.behavior_id = zlib.adler32(be_content_new.encode()) & 0x7fffffff

            # reset state before starting new behavior
            self._current_state = None
            self._behavior_started = False
            self._preempt_requested = False
            self._requested_behavior_id = be_selection.behavior_id
            self._active_behavior_id = None
            self.running = True
            self.outcome = ''

            # start new behavior
            self._pub.publish(be_selection)
            self._current_goal.execute()
        except Exception as exc:  # noqa: B902
            self._node.get_logger().error(f'Failed to initialize accepted goal for behavior "{goal.behavior_name}": {exc}')
            if self._has_active_goal():
                self._current_goal.abort()
            self.clean_me()

    def _preempt_cb(self, goal_handle):
        # pylint: disable=unused-argument
        self._preempt_requested = True
        if not self._behavior_started:
            return CancelResponse.ACCEPT
        # Send the preempt request to real behavior
        self._preempt_pub.publish(Empty())
        self._node.get_logger().info('Behavior execution preempt requested!')
        return CancelResponse.ACCEPT

    def clean_me(self):
        """Clean up flags."""
        self.running = False
        self._current_state = None
        self._behavior_started = False
        self._preempt_requested = False
        self._requested_behavior_id = None
        self._active_behavior_id = None

    def _has_active_goal(self):
        """Return True if there is a goal handle that can still receive terminal updates."""
        return self._current_goal is not None and self._current_goal.is_active

    def _execute_cb(self, goal_handle):
        self._node.get_logger().info('Executing behavior')

        while rclpy.ok() and self.running:
            time.sleep(0.01)
        print('End execution')
        self.clean_me()
        result = BehaviorExecution.Result()
        result.outcome = self.outcome
        self.outcome = ''
        return result

    def _status_cb(self, msg):
        if not self._behavior_started and msg.code == BEStatus.STARTED and msg.behavior_id == self._requested_behavior_id:
            self._behavior_started = True
            self._active_behavior_id = msg.behavior_id
            self._requested_behavior_id = None
            self._node.get_logger().info('Behavior execution has started!')
            # Preempt if the goal was asked to preempt before the behavior started
            if self._preempt_requested:
                self._preempt_cb(self._current_goal)

        if not self._behavior_started:
            prestart_outcomes = {
                BEStatus.ERROR: 'error',
                BEStatus.FAILED: 'failed',
            }
            if msg.code in prestart_outcomes and msg.behavior_id == self._requested_behavior_id:
                self._node.get_logger().error('Requested behavior failed before startup completed '
                                              f'(status={msg.code})!')
                if self._current_goal is not None and self._current_goal.is_active:
                    self._current_goal.abort()
                self.outcome = prestart_outcomes[msg.code]
                self.clean_me()
            elif msg.code in prestart_outcomes:
                self._node.get_logger().warning(f'Ignored status={msg.code} before startup because behavior id '
                                                f'differed ({msg.behavior_id} vs {self._requested_behavior_id})!')
            elif msg.code == BEStatus.WARNING:
                self._node.get_logger().warning('Ignored non-terminal WARNING before startup completed '
                                                f'for behavior id={msg.behavior_id}.')
            return

        if msg.behavior_id != self._active_behavior_id:
            self._node.get_logger().warning('Ignored status because behavior id differed '
                                            f'({msg.behavior_id} vs {self._active_behavior_id})!')
            return
        elif msg.code == BEStatus.ERROR:
            self._node.get_logger().error('Failed to run behavior! Check onboard terminal for further infos.')
            if self._has_active_goal():
                self._current_goal.abort()
            self.outcome = 'error'
            self.clean_me()
        elif msg.code == BEStatus.FINISHED:
            result = msg.args[0] if len(msg.args) >= 1 else ''
            self._node.get_logger().info("Finished behavior execution with result '%s'!" % result)
            if result == 'preempted':
                if self._has_active_goal():
                    self._current_goal.canceled()
                self.outcome = 'preempted'
                self.clean_me()
            else:
                if self._has_active_goal():
                    self._current_goal.succeed()
                self.outcome = 'success'
                self.clean_me()
        elif msg.code == BEStatus.FAILED:
            self._node.get_logger().error("Behavior execution failed in state '%s'!" % str(self._current_state))
            if self._has_active_goal():
                self._current_goal.abort()
            self.outcome = 'failed'
            self.clean_me()
        elif msg.code == BEStatus.WARNING:
            self._node.get_logger().warning("Ignoring non-terminal WARNING for active behavior id=%s in state '%s'."
                                            % (msg.behavior_id, str(self._current_state)))

    def _state_cb(self, msg):
        self._current_state = msg.data
        if self._current_goal and self._current_goal.is_active:
            self._current_goal.publish_feedback(BehaviorExecution.Feedback(current_state=self._current_state))
            self._node.get_logger().info('Current state id = %d' % self._current_state)


def main(args=None):
    """Start the behavior action server node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = rclpy.create_node('flexbe_action_server')
    BehaviorActionServer(node)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
