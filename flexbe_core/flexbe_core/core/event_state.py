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


"""EventState."""

from flexbe_core.core.operatable_state import OperatableState
from flexbe_core.core.preemptable_state import PreemptableState
from flexbe_core.core.priority_container import PriorityContainer
from flexbe_core.core.topics import Topics
from flexbe_core.logger import Logger
from flexbe_core.state_logger import StateLogger

from flexbe_msgs.msg import CommandFeedback

from std_msgs.msg import Bool, Empty


@StateLogger.log_events('flexbe.events',
                        start='on_start', stop='on_stop',
                        pause='on_pause', resume='on_resume',
                        enter='on_enter', exit='on_exit')
@StateLogger.log_userdata('flexbe.userdata')
class EventState(OperatableState):
    """A state that allows implementing certain events."""

    def __init__(self, *args, **kwargs):
        """Initialize EventState instance."""
        super().__init__(*args, **kwargs)

        self.__execute = self.execute
        self.execute = self._event_execute

        self._skipped = False
        self._paused = False
        self._last_active_container = None
        self._last_outcome = None

    def _event_execute(self, *args, **kwargs):
        if self._is_controlled and self._sub.has_msg(Topics._CMD_PAUSE_TOPIC):
            msg = self._sub.get_last_msg(Topics._CMD_PAUSE_TOPIC)
            self._sub.remove_last_msg(Topics._CMD_PAUSE_TOPIC)
            if msg.data:
                Logger.localinfo("--> Pausing in state '%s'", self.name)
                self._pub.publish(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback(command='pause'))
                self._last_active_container = PriorityContainer.active_container
                # claim priority to propagate pause event
                PriorityContainer.active_container = self.path
                self._paused = True
            else:
                Logger.localinfo("--> Resuming in state '%s'", self.name)
                self._pub.publish(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback(command='resume'))
                PriorityContainer.active_container = self._last_active_container
                self._last_active_container = None
                self._paused = False

        if self._paused and not PreemptableState.preempt:
            self._notify_skipped()
            return None

        if self._entering:
            self._entering = False
            self._exited = False
            self._last_outcome = None
            self.on_enter(*args, **kwargs)

        if self._skipped and not PreemptableState.preempt:
            self._skipped = False
            self.on_resume(*args, **kwargs)

        self._last_execution = EventState._node.get_clock().now()
        outcome = self.__execute(*args, **kwargs)

        repeat = False
        if self._is_controlled and self._sub.has_msg(Topics._CMD_REPEAT_TOPIC):
            Logger.localinfo("--> Repeating state '%s'", self.name)
            self._sub.remove_last_msg(Topics._CMD_REPEAT_TOPIC)
            self._pub.publish(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback(command='repeat'))
            repeat = True

        if repeat or outcome is not None:
            # As this is currently coded, a repeat command will immediately halt
            # call on_exit, then reenter the state
            # (vs. an alternative to wait until outcome and then repeat)
            self.on_exit(*args, **kwargs)
            self._exited = True
            self._entering = True  # for next call
            if repeat:
                outcome = None  # clear outcome so we stay on this state
            else:
                # Publish outcome for this state
                self._publish_outcome(outcome)

        self._last_outcome = outcome
        return outcome

    def _notify_skipped(self):
        if not self._skipped:
            self.on_pause()
            self._skipped = True
        super()._notify_skipped()

    def _enable_ros_control(self):
        if not self._is_controlled:
            super()._enable_ros_control()
            self._pub.create_publisher(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback)
            self._sub.subscribe(Topics._CMD_REPEAT_TOPIC, Empty, inst_id=id(self))
            self._sub.subscribe(Topics._CMD_PAUSE_TOPIC, Bool, inst_id=id(self))

    def _disable_ros_control(self):
        if self._is_controlled:
            super()._disable_ros_control()
            self._pub.remove_publisher(Topics._CMD_FEEDBACK_TOPIC)
            self._sub.unsubscribe_topic(Topics._CMD_REPEAT_TOPIC, inst_id=id(self))
            self._sub.unsubscribe_topic(Topics._CMD_PAUSE_TOPIC, inst_id=id(self))
            self._last_active_container = None
            if self._paused:
                PriorityContainer.active_container = None

    # Events
    # (just implement the ones you need)

    def on_start(self):
        """Execute once when the behavior starts."""

    def on_stop(self):
        """Execute once when the behavior stops or is preempted."""

    def on_pause(self):
        """Execute each time this state is paused."""

    def on_resume(self, userdata):
        """Execute each time this state is resumed."""

    def on_enter(self, userdata):
        """Execute each time the state is entered from any other state (but not from itself)."""

    def on_exit(self, userdata):
        """Execute each time the state will be left to any other state (but not to itself)."""
