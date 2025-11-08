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


"""Implement LockableState that can prevent transition."""

from flexbe_core.core.manually_transitionable_state import ManuallyTransitionableState
from flexbe_core.core.topics import Topics
from flexbe_core.logger import Logger

from flexbe_msgs.msg import CommandFeedback

from std_msgs.msg import Int32


class LockableState(ManuallyTransitionableState):
    """
    A state that can be locked.

    When locked, no transition can be done regardless of the resulting outcome.
    However, if any outcome would be triggered, the outcome will be stored
    and the state won't be executed anymore until it is unlocked and the stored outcome is set.
    """

    def __init__(self, *args, **kwargs):
        """Initialize LockableState instance."""
        super().__init__(*args, **kwargs)
        self.__execute = self.execute
        self.execute = self._lockable_execute

        self._locked = False
        self._stored_outcome = None

    def _lockable_execute(self, *args, **kwargs):
        """Execute lockable portion of state."""
        if self._is_controlled and self._sub.has_msg(Topics._CMD_LOCK_TOPIC):
            msg = self._sub.get_last_msg(Topics._CMD_LOCK_TOPIC)
            self._sub.remove_last_msg(Topics._CMD_LOCK_TOPIC)
            self._execute_lock(msg.data)

        if self._is_controlled and self._sub.has_msg(Topics._CMD_UNLOCK_TOPIC):
            msg = self._sub.get_last_msg(Topics._CMD_UNLOCK_TOPIC)
            self._sub.remove_last_msg(Topics._CMD_UNLOCK_TOPIC)
            self._execute_unlock(msg.data)

        # locked, so execute until we want to transition
        if self._locked:
            if self._stored_outcome is None or self._stored_outcome == 'None':
                self._stored_outcome = self.__execute(*args, **kwargs)
            return None

        # not locked, but there still is a transition we want to trigger
        if not self._locked and self._stored_outcome is not None and not self._stored_outcome == 'None':
            if self.parent.transition_allowed(self.name, self._stored_outcome):
                outcome = self._stored_outcome
                self._stored_outcome = None
                return outcome

            return None

        outcome = self.__execute(*args, **kwargs)

        if outcome is None or outcome == 'None':
            return None

        # not locked, but still, a parent could be locked so we need to ensure that we do not cause an outcome there
        if self.parent is not None and not self.parent.transition_allowed(self.name, outcome):
            self._stored_outcome = outcome
            return None

        return outcome

    def _execute_lock(self, target):
        """Execute lock."""
        if target in (self.state_id, 0):
            target = self.state_id
            found_target = True
            self._locked = True
        else:
            found_target = self._parent.lock(target)
        # provide feedback about lock
        self._pub.publish(Topics._CMD_FEEDBACK_TOPIC,
                          CommandFeedback(command='lock', args=[f'{target}', f'{target}' if found_target else '-1']))
        if not found_target:
            Logger.logwarn(f"Wanted to lock state id '{target}', but could not find it in current path '{self.path}'.")
        else:
            Logger.localinfo(f"--> Locking in state '{target}' at '{self.path}'")

    def _execute_unlock(self, target):
        if target == self.state_id or (self._locked and target == 0):
            target = self.state_id
            found_target = True
            self._locked = False
        else:
            found_target = self._parent.unlock(target)
        # provide feedback about unlock
        self._pub.publish(Topics._CMD_FEEDBACK_TOPIC,
                          CommandFeedback(command='unlock', args=[f'{target}', f'{target}' if found_target else '-1']))
        if not found_target:
            Logger.logwarn(f"Wanted to unlock '{target}', but could not find it in current path '{self.path}'.")
        else:
            Logger.localinfo(f"--> Unlocking in state '{target}' at '{self.path}'")

    def _enable_ros_control(self):
        if not self._is_controlled:
            super()._enable_ros_control()
            self._pub.create_publisher(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback)
            self._sub.subscribe(Topics._CMD_LOCK_TOPIC, Int32, inst_id=id(self))
            self._sub.subscribe(Topics._CMD_UNLOCK_TOPIC, Int32, inst_id=id(self))

    def _disable_ros_control(self):
        if self._is_controlled:
            super()._disable_ros_control()
            self._pub.remove_publisher(Topics._CMD_FEEDBACK_TOPIC)
            self._sub.unsubscribe_topic(Topics._CMD_LOCK_TOPIC, inst_id=id(self))
            self._sub.unsubscribe_topic(Topics._CMD_UNLOCK_TOPIC, inst_id=id(self))

    def is_locked(self):
        """Check is locked."""
        return self._locked
