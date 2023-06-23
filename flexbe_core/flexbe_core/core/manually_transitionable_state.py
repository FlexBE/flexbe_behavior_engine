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


"""ManuallyTransitionableState."""

from flexbe_msgs.msg import CommandFeedback, OutcomeRequest

from flexbe_core.core.ros_state import RosState
from flexbe_core.logger import Logger


class ManuallyTransitionableState(RosState):
    """
    A state for that a desired outcome can be declared.

    If any outcome is declared, this outcome is forced.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__execute = self.execute
        self.execute = self._manually_transitionable_execute

        self._force_transition = False
        self._manual_transition_requested = None
        self._feedback_topic = 'flexbe/command_feedback'
        self._transition_topic = 'flexbe/command/transition'

    def _manually_transitionable_execute(self, *args, **kwargs):
        self._manual_transition_requested = None
        if self._is_controlled and self._sub.has_buffered(self._transition_topic):
            command_msg = self._sub.get_from_buffer(self._transition_topic)
            self._pub.publish(self._feedback_topic, CommandFeedback(command="transition",
                                                                    args=[command_msg.target, self.name]))
            if command_msg.target != self.name:
                Logger.logwarn("Requested outcome for state %s but active state is %s" %
                               (command_msg.target, self.name))
            else:
                self._force_transition = True
                outcome = self.outcomes[command_msg.outcome]
                self._manual_transition_requested = outcome
                Logger.localinfo("--> Manually triggered outcome %s of state %s" % (outcome, self.name))
                return outcome
        # otherwise, return the normal outcome
        self._force_transition = False
        return self.__execute(*args, **kwargs)

    def _enable_ros_control(self):
        super()._enable_ros_control()
        self._pub.createPublisher(self._feedback_topic, CommandFeedback)
        self._sub.subscribe(self._transition_topic, OutcomeRequest, inst_id=id(self))
        self._sub.enable_buffer(self._transition_topic)

    def _disable_ros_control(self):
        super()._disable_ros_control()
        self._sub.unsubscribe_topic(self._transition_topic, inst_id=id(self))
