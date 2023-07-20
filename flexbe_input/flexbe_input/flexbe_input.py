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


"""
Enable operator inputs to onboard behavior.

Created on 02/13/2015

@author: Philipp Schillinger, Brian Wright
"""

from rclpy.action import ActionClient

from flexbe_msgs.action import BehaviorInput
from flexbe_core import Logger
from .complex_action_server import ComplexActionServer


class FlexBEInput:
    """Enable operator inputs to onboard behavior."""

    def __init__(self, node):
        """Provide relay from onboard to OCS."""
        # onboard connection
        self._node = node
        self._as = ComplexActionServer(node=node,
                                       name='flexbe/behavior_input',
                                       ActionSpec=BehaviorInput,
                                       execute_cb=self.execute_cb,
                                       auto_start=False)

        Logger.loginfo("Ready for data requests...")

    def execute_cb(self, goal, goal_handle):
        Logger.loginfo("--> Got a request!")
        Logger.loginfo(f"'{goal.msg}'")

        relay_ocs_client_ = ActionClient(self._node,
                                         BehaviorInput,
                                         'flexbe/operator_input')

        # wait for data msg
        Logger.localinfo("FlexBEInput waiting to relay from OCS ...")
        relay_ocs_client_.wait_for_server()
        Logger.localinfo("FlexBEInput is ready!")

        # Fill in the goal here
        result = relay_ocs_client_.send_goal(goal)  # This is a blocking call!

        # result.data now serialized
        data_str = result.data
        Logger.localinfo(f"FlexBEInput result={data_str}")

        if result.result_code == BehaviorInput.Result.RESULT_OK:
            self._as.set_succeeded(BehaviorInput.Result(result_code=BehaviorInput.Result.RESULT_OK,
                                                        data=data_str), "ok", goal_handle)

        elif result.result_code == BehaviorInput.Result.RESULT_FAILED:
            # remove
            self._as.set_succeeded(BehaviorInput.Result(result_code=BehaviorInput.Result.RESULT_FAILED,
                                                        data=data_str), "failed", goal_handle)
            Logger.loginfo("<-- Replied with FAILED")

        elif result.result_code == BehaviorInput.Result.RESULT_ABORTED:
            self._as.set_succeeded(BehaviorInput.Result(result_code=BehaviorInput.Result.RESULT_ABORTED,
                                                        data=data_str), "Aborted", goal_handle)
            Logger.loginfo("<-- Replied with ABORT")
