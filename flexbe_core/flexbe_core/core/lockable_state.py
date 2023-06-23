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


"""Implement LockableState that can prevent transition."""
from std_msgs.msg import String

from flexbe_msgs.msg import CommandFeedback

from flexbe_core.logger import Logger
from flexbe_core.core.manually_transitionable_state import ManuallyTransitionableState


class LockableState(ManuallyTransitionableState):
    """
    A state that can be locked.

    When locked, no transition can be done regardless of the resulting outcome.
    However, if any outcome would be triggered, the outcome will be stored
    and the state won't be executed anymore until it is unlocked and the stored outcome is set.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__execute = self.execute
        self.execute = self._lockable_execute

        self._locked = False
        self._stored_outcome = None

        self._feedback_topic = 'flexbe/command_feedback'
        self._lock_topic = 'flexbe/command/lock'
        self._unlock_topic = 'flexbe/command/unlock'

    def _lockable_execute(self, *args, **kwargs):
        if self._is_controlled and self._sub.has_msg(self._lock_topic):
            msg = self._sub.get_last_msg(self._lock_topic)
            self._sub.remove_last_msg(self._lock_topic)
            self._execute_lock(msg.data)

        if self._is_controlled and self._sub.has_msg(self._unlock_topic):
            msg = self._sub.get_last_msg(self._unlock_topic)
            self._sub.remove_last_msg(self._unlock_topic)
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
        if target in (self.path, ''):
            target = self.path
            found_target = True
            self._locked = True
        else:
            found_target = self._parent.lock(target)
        # provide feedback about lock
        self._pub.publish(self._feedback_topic, CommandFeedback(command="lock",
                                                                args=[target, target if found_target else ""]))
        if not found_target:
            Logger.logwarn(f"Wanted to lock {target}, but could not find it in current path {self.path}.")
        else:
            Logger.localinfo(f"--> Locking in state {target}")

    def _execute_unlock(self, target):
        if target == self.path or (self._locked and target == ''):
            target = self.path
            found_target = True
            self._locked = False
        else:
            found_target = self._parent.unlock(target)
        # provide feedback about unlock
        self._pub.publish(self._feedback_topic, CommandFeedback(command="unlock",
                                                                args=[target, target if found_target else ""]))
        if not found_target:
            Logger.logwarn(f"Wanted to unlock {target}, but could not find it in current path {self.path}.")
        else:
            Logger.localinfo(f"--> Unlocking in state {target}")

    def _enable_ros_control(self):
        super()._enable_ros_control()
        self._pub.createPublisher(self._feedback_topic, CommandFeedback)
        self._sub.subscribe(self._lock_topic, String, inst_id=id(self))
        self._sub.subscribe(self._unlock_topic, String, inst_id=id(self))

    def _disable_ros_control(self):
        super()._disable_ros_control()
        self._sub.unsubscribe_topic(self._lock_topic, inst_id=id(self))
        self._sub.unsubscribe_topic(self._unlock_topic, inst_id=id(self))

    def is_locked(self):
        return self._locked
