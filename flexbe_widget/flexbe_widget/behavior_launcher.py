#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from flexbe_msgs.msg import *
from rosidl_runtime_py import get_interface_path

from flexbe_core import BehaviorLibrary
from std_msgs.msg import String

import pickle
import zlib
import difflib
import os
import yaml
import xml.etree.ElementTree as ET
import threading

class BehaviorLauncher(object):

	MIN_VERSION = '2.2.0'

	def __init__(self, node):
		self._node = node
		self._ready_event = threading.Event()

		self._sub = self._node.create_subscription(BehaviorRequest, "flexbe/request_behavior", self._callback, 100)
		self._version_sub = self._node.create_subscription(String, "flexbe/ui_version", self._version_callback, 100)

		self._pub = self._node.create_publisher(BehaviorSelection, "flexbe/start_behavior", 100)
		self._status_pub = self._node.create_publisher(BEStatus, "flexbe/status", 100)
		self._status_sub = self._node.create_subscription(BEStatus, "flexbe/status", self._status_callback, 100)
		self._mirror_pub = self._node.create_publisher(ContainerStructure, "flexbe/mirror/structure", 100)

		# self._rp = RosPack()
		self._behavior_lib = BehaviorLibrary(node)

		self._node.get_logger().info("%d behaviors available, ready for start request." % self._behavior_lib.count_behaviors())

	def _status_callback(self, msg):
		if msg.code in [BEStatus.READY, BEStatus.FINISHED, BEStatus.FAILED, BEStatus.ERROR]:
			self._ready_event.set()

	def _callback(self, msg):
		self._node.get_logger().info("Got message from request behavior")
		be_id, behavior = self._behavior_lib.find_behavior(msg.behavior_name)
		if be_id is None:
			self._node.get_logger().error("Did not find behavior with requested name: %s" % msg.behavior_name)
			self._status_pub.publish(BEStatus(code=BEStatus.ERROR))
			return

		self._node.get_logger().info("Request for behavior " + str(behavior["name"]))

		be_selection = BehaviorSelection()
		be_selection.behavior_id = be_id
		be_selection.autonomy_level = msg.autonomy_level
		try:
			for k, v in zip(msg.arg_keys, msg.arg_values):
				if k.startswith('/YAML:'):
					key = k.replace('/YAML:', '/', 1)
					path = v.split(':')[0]
					ns = v.split(':')[1]
					if path.startswith('~') or path.startswith('/'):
						yamlpath = os.path.expanduser(path)
					else:
						yamlpath = os.path.join(get_interface_path(path.split('/')[0]), '/'.join(path.split('/')[1:]))
						# yamlpath = os.path.join(self._rp.get_path(path.split('/')[0]), '/'.join(path.split('/')[1:]))
					with open(yamlpath, 'r') as f:
						content = getattr(yaml, 'unsafe_load', yaml.load)(f)
					if ns != '' and ns in content:
						content = content[ns]
					be_selection.arg_keys.append(key)
					be_selection.arg_values.append(yaml.dump(content))
				else:
					be_selection.arg_keys.append(k)
					be_selection.arg_values.append(v)
		except Exception as e:
			self._node.get_logger().warn('Failed to parse and substitute behavior arguments, will use direct input.\n%s' % str(e))
			be_selection.arg_keys = msg.arg_keys
			be_selection.arg_values = msg.arg_values

		# wait until Behavior Engine status is BEStatus.READY
		self._ready_event.wait()

		be_structure = ContainerStructure()
		be_structure.containers = msg.structure

		try:
			be_filepath_new = self._behavior_lib.get_sourcecode_filepath(be_id)
		except Exception as e:
			self._node.get_logger().error("Could not find behavior package '%s'" % (behavior["package"]))
			self._node.get_logger().info("Have you updated your ROS_PACKAGE_PATH after creating the behavior?")
			return

		with open(be_filepath_new, "r") as f:
			be_content_new = f.read()

		be_filepath_old = self._behavior_lib.get_sourcecode_filepath(be_id, add_tmp=True)
		if not os.path.isfile(be_filepath_old):
			be_selection.behavior_checksum = zlib.adler32(be_content_new.encode()) & 0x7fffffff
			if msg.autonomy_level != 255:
				be_structure.behavior_id = be_selection.behavior_checksum
				self._mirror_pub.publish(be_structure)
			self._pub.publish(be_selection)
			self._node.get_logger().info("No changes to behavior version.")
			return

		with open(be_filepath_old, "r") as f:
			be_content_old = f.read()

		sqm = difflib.SequenceMatcher(a=be_content_old, b=be_content_new)
		diffs = [x[1] for x in sqm.get_grouped_opcodes(0)]
		for opcode, a0, a1, b0, b1 in diffs:
			content = be_content_new[b0:b1]
			be_selection.modifications.append(BehaviorModification(a0, a1, content))

		be_selection.behavior_checksum = zlib.adler32(be_content_new.encode()) & 0x7fffffff
		if msg.autonomy_level != 255:
			be_structure.behavior_id = be_selection.behavior_checksum
			self._mirror_pub.publish(be_structure)

		self._pub.publish(be_selection)

	def _version_callback(self, msg):
		vui = self._parse_version(msg.data)
		vex = self._parse_version(BehaviorLauncher.MIN_VERSION)
		if vui < vex:
			self._node.get_logger().warn('FlexBE App needs to be updated!\n' \
				+ 'Require at least version %s, but have %s\n' % (BehaviorLauncher.MIN_VERSION, msg.data) \
				+ 'Please run a "git pull" in "roscd flexbe_app".')

	def _parse_version(self, v):
		result = 0
		offset = 1
		for n in reversed(v.split('.')):
			result += int(n) * offset
			offset *= 100
		return result
