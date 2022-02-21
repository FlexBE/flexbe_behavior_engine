#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rosidl_runtime_py import get_interface_path

from flexbe_msgs.msg import *
from flexbe_msgs.action import *
from flexbe_core import BehaviorLibrary

from std_msgs.msg import String, Empty

import zlib
import difflib
import os
import yaml
import xml.etree.ElementTree as ET


class BehaviorActionServer(object):

	def __init__(self, node):
		self._node = node
		self._behavior_started = False
		self._preempt_requested = False
		self._current_goal = None

		self._current_state = None
		self._active_behavior_id = None

		self._pub = self._node.create_publisher(BehaviorSelection, 'flexbe/start_behavior', 100)
		self._preempt_pub = self._node.create_publisher(Empty, 'flexbe/command/preempt', 100)
		self._status_pub = self._node.create_subscription(BEStatus, 'flexbe/status', self._status_cb, 100)
		self._state_pub = self._node.create_subscription(String, 'flexbe/behavior_update', self._state_cb, 100)

		self._as = ActionServer(self._node, BehaviorExecution, 'flexbe/execute_behavior', goal_callback=self._goal_cb,
								cancel_callback=self._preempt_cb, execute_callback=self._execute_cb)

		self._behavior_lib = BehaviorLibrary(node)

		self._node.get_logger().info("%d behaviors available, ready for start request." % self._behavior_lib.count_behaviors())


	def _goal_cb(self, goal_handle):
		self._current_goal = goal_handle
		goal = goal_handle.request()

		if self._preempt_requested:
			goal_handle.canceled()

		self._node.get_logger().info('Received a new request to start behavior: %s' % goal.behavior_name)
		be_id, behavior = self._behavior_lib.find_behavior(goal.behavior_name)
		if be_id is None:
			self._node.get_logger().error("Deny goal: Did not find behavior with requested name %s" % goal.behavior_name)
			self._current_goal.canceled()
			return

		be_selection = BehaviorSelection()
		be_selection.behavior_id = be_id
		be_selection.autonomy_level = 255
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
						content = getattr(yaml, 'unsafe_load', yaml.load)(content)
						if ns in content:
							content = content[ns]
						content = yaml.dump(content)
					be_selection.arg_keys.append(k)
					be_selection.arg_values.append(content)
				else:
					be_selection.arg_keys.append(k)
					be_selection.arg_values.append(v)
		except Exception as e:
			self._node.get_logger().warn('Failed to parse and substitute behavior arguments, will use direct input.\n%s' % str(e))
			be_selection.arg_keys = goal.arg_keys
			be_selection.arg_values = goal.arg_values
		be_selection.input_keys = goal.input_keys
		be_selection.input_values = goal.input_values

		# check for local modifications of the behavior to send them to the onboard behavior
		be_filepath_new = self._behavior_lib.get_sourcecode_filepath(be_id)
		with open(be_filepath_new, "r") as f:
			be_content_new = f.read()

		be_filepath_old = self._behavior_lib.get_sourcecode_filepath(be_id, add_tmp=True)
		if not os.path.isfile(be_filepath_old):
			be_selection.behavior_checksum = zlib.adler32(be_content_new.encode()) & 0x7fffffff
		else:
			with open(be_filepath_old, "r") as f:
				be_content_old = f.read()

			sqm = difflib.SequenceMatcher(a=be_content_old, b=be_content_new)
			diffs = [x[1] for x in sqm.get_grouped_opcodes(0)]
			for opcode, a0, a1, b0, b1 in diffs:
				content = be_content_new[b0:b1]
				be_selection.modifications.append(BehaviorModification(a0, a1, content))

			be_selection.behavior_checksum = zlib.adler32(be_content_new.encode()) & 0x7fffffff

		# reset state before starting new behavior
		self._current_state = None
		self._behavior_started = False
		self._preempt_requested = False

		# start new behavior
		self._pub.publish(be_selection)


	def _preempt_cb(self, goal_handle):
		self._preempt_requested = True
		if not self._behavior_started:
			return
		self._preempt_pub.publish(Empty())
		self._node.get_logger().info('Behavior execution preempt requested!')


	def _execute_cb(self, goal_handle):
		self._node.get_logger().info("Executing behavior")


	def _status_cb(self, msg):
		if msg.code == BEStatus.ERROR:
			self._node.get_logger().error('Failed to run behavior! Check onboard terminal for further infos.')
			self._current_goal.abort()
			return
		if not self._behavior_started and msg.code == BEStatus.STARTED:
			self._behavior_started = True
			self._active_behavior_id = msg.behavior_id
			self._node.get_logger().info('Behavior execution has started!')
			# Preempt if the goal was asked to preempt before the behavior started
			if self._preempt_requested:
				self._preempt_cb()
		# Ignore status until behavior start was received
		if not self._behavior_started:
			return

		if msg.behavior_id != self._active_behavior_id:
			self._node.get_logger().warn('Ignored status because behavior id differed ({} vs {})!'.format(msg.behavior_id, self._active_behavior_id))
			return
		elif msg.code == BEStatus.FINISHED:
			result = msg.args[0] if len(msg.args) >= 1 else ''
			self._node.get_logger().info('Finished behavior execution with result "%s"!' % result)
			self._current_goal.succeed()
		elif msg.code == BEStatus.FAILED:
			self._node.get_logger().error('Behavior execution failed in state %s!' % str(self._current_state))
			self._current_goal.abort()


	def _state_cb(self, msg):
		self._current_state = msg.data
		if self._as.is_active:
			self._current_goal.publish_feedback(BehaviorExecutionFeedback(self._current_state))
			self._node.get_logger().loginfo('Current state: %s' % self._current_state)
