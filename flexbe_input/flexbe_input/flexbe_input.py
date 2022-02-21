#!/usr/bin/env python

import pickle
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from flexbe_msgs.msg import BehaviorInputAction, BehaviorInputFeedback, BehaviorInputResult, BehaviorInputGoal
from flexbe_core import Logger
from .complex_action_server import ComplexActionServer
'''
Created on 02/13/2015

@author: Philipp Schillinger, Brian Wright
'''

class BehaviorInput(object):

	def __init__(self, node):
		'''
		Constructor
		'''
		#onboard connection
		self._node = node
		self._as = ComplexActionServer('flexbe/behavior_input', BehaviorInputAction, execute_cb=self.execute_cb, auto_start = False)

		Logger.loginfo("Ready for data requests...")

	def execute_cb(self, goal , goal_handle):
		Logger.loginfo("--> Got a request!")
		Logger.loginfo('"%s"' % goal.msg)

		relay_ocs_client_ = ActionClient(self._node, BehaviorInputAction, 'flexbe/operator_input')

		# wait for data msg
		print("waiting")
		relay_ocs_client_.wait_for_server()
		print("done")

		# Fill in the goal here
		relay_ocs_client_.send_goal(goal)

		result = BehaviorInputResult()
		result = relay_ocs_client_.get_result()

		#result.data now serialized
		data_str = result.data
		print(data_str)

		if(result.result_code == BehaviorInputResult.RESULT_OK):
			self._as.set_succeeded(BehaviorInputResult(result_code=BehaviorInputResult.RESULT_OK, data=data_str), "ok",goal_handle)

		elif(result.result_code == BehaviorInputResult.RESULT_FAILED):
			# remove
			self._as.set_succeeded(BehaviorInputResult(result_code=BehaviorInputResult.RESULT_FAILED, data=data_str),"failed",goal_handle)
			Logger.loginfo("<-- Replied with FAILED")

		elif(result.result_code == BehaviorInputResult.RESULT_ABORTED ):
			self._as.set_succeeded(BehaviorInputResult(result_code=BehaviorInputResult.RESULT_ABORTED, data=data_str),"Aborted",goal_handle)
			Logger.loginfo("<-- Replied with ABORT")
