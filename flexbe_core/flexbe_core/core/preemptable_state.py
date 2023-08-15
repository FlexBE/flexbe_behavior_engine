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


"""PreemptableState."""

from std_msgs.msg import Empty

from flexbe_msgs.msg import CommandFeedback

from flexbe_core.core.lockable_state import LockableState
from flexbe_core.core.topics import Topics
from flexbe_core.logger import Logger


class PreemptableState(LockableState):
    """
    A state that can be preempted.

    If preempted, the state will not be executed anymore and return the outcome preempted.
    """

    preempt = False

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__execute = self.execute
        self.execute = self._preemptable_execute

        PreemptableState.preempt = False

    def _preemptable_execute(self, *args, **kwargs):
        if self._is_controlled and self._sub.has_msg(Topics._CMD_PREEMPT_TOPIC):
            self._sub.remove_last_msg(Topics._CMD_PREEMPT_TOPIC)
            self._pub.publish(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback(command="preempt"))
            PreemptableState.preempt = True
            Logger.localinfo("--> Behavior will be preempted")

        if PreemptableState.preempt:
            if not self._is_controlled:
                Logger.localinfo("Behavior will be preempted")
            self._force_transition = True
            return self._preempted_name

        return self.__execute(*args, **kwargs)

    def _notify_skipped(self):
        # make sure we dont miss a preempt even if not being executed
        if self._is_controlled and self._sub.has_msg(Topics._CMD_PREEMPT_TOPIC):
            self._sub.remove_last_msg(Topics._CMD_PREEMPT_TOPIC)
            self._pub.publish(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback(command="preempt"))
            PreemptableState.preempt = True

    def _enable_ros_control(self):
        if not self._is_controlled:
            super()._enable_ros_control()
            self._pub.create_publisher(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback)
            self._sub.subscribe(Topics._CMD_PREEMPT_TOPIC, Empty, inst_id=id(self))
            PreemptableState.preempt = False

    def _disable_ros_control(self):
        if self._is_controlled:
            super()._disable_ros_control()
            self._sub.unsubscribe_topic(Topics._CMD_PREEMPT_TOPIC, inst_id=id(self))
            self._pub.remove_publisher(Topics._CMD_FEEDBACK_TOPIC)
