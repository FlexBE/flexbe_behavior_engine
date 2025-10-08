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


"""A state machine that can be preempted."""
import threading

from flexbe_core.core.lockable_state_machine import LockableStateMachine
from flexbe_core.core.preemptable_state import PreemptableState
from flexbe_core.core.state import State
from flexbe_core.core.topics import Topics
from flexbe_core.logger import Logger

from flexbe_msgs.msg import CommandFeedback

import rclpy

from std_msgs.msg import Empty


class PreemptableStateMachine(LockableStateMachine):
    """
    A state machine that can be preempted.

    If preempted, the state machine will return the outcome preempted.
    """

    def __init__(self, *args, **kwargs):
        """Initialize instance."""
        super().__init__(*args, **kwargs)
        self._status_lock = threading.Lock()
        self._last_deep_states_list = None
        self._last_outcome = None

    def _notify_start(self):
        # always listen to preempt so that the behavior can be stopped even if unsupervised (e.g not ROS controlled via OCS)
        self._sub.subscribe(Topics._CMD_PREEMPT_TOPIC, Empty, self._preempt_cb, inst_id=id(self))

    def _notify_stop(self):
        self._sub.unsubscribe_topic(Topics._CMD_PREEMPT_TOPIC, inst_id=id(self))

    def _preempt_cb(self, msg):
        if not self._is_controlled:
            Logger.localinfo(f'Preempting {self.name}!')
            PreemptableState.preempt = True

    @staticmethod
    def add(label, state, transitions=None, remapping=None):
        """Add state to SM."""
        transitions[State._preempted_name] = State._preempted_name
        LockableStateMachine.add(label, state, transitions, remapping)

    @property
    def _valid_targets(self):
        return super()._valid_targets + [State._preempted_name]

    def spin(self, userdata=None, rclpy_context=None):
        """Spin the execute loop for preemptable portion."""
        outcome = None
        while rclpy.ok(context=rclpy_context):
            command_msg = self._sub.peek_at_buffer(Topics._CMD_TRANSITION_TOPIC)

            try:
                outcome = self.execute(userdata)
            except Exception as exc:
                Logger.logerr(f"Exception in '{self.name}' - stopping behavior!")
                Logger.localinfo(f'{exc}')
                self.on_exit(userdata)  # Call to preempt any active states
                Logger.logerr(f"Exception in '{self.name}' - {exc}")
                return None

            if command_msg is not None:
                command_msg2 = self._sub.peek_at_buffer(Topics._CMD_TRANSITION_TOPIC)
                if command_msg is command_msg2:
                    # Execute loop went through process and did not handle the requested transition
                    Logger.loginfo(f"'{self.name}' did not handle transition "
                                   f"cmd='{command_msg.target}' ({command_msg.outcome}) - toss it!")
                    self._pub.publish(Topics._CMD_FEEDBACK_TOPIC,
                                      CommandFeedback(command='transition',
                                                      args=['invalid', f'{command_msg.target}']))
                    self._sub.get_from_buffer(Topics._CMD_TRANSITION_TOPIC)

            # Store the information for safely passing to heartbeat thread
            deep_states = self.get_deep_states()
            assert isinstance(deep_states, list), f'Expecting a list here, not {deep_states}'
            if deep_states != self._last_deep_states_list:
                # Logger.localinfo(f"New deep states for '{self.name}' len={len(deep_states)} "
                #                  f'deep states: {[dpst.path for dpst in deep_states if dpst is not None]}')
                with self._status_lock:
                    self._last_deep_states_list = deep_states
            # else:
            #     Logger.localinfo(f"Same old deep states for '{self.name}' len={len(deep_states)} - "
            #                      f'deep states: {[dpst.name for dpst in deep_states]}')

            if self._inner_sync_request:
                # Top-level state machine with sync request
                self.process_sync_request()

            if outcome is not None:
                Logger.loginfo(f"PreemptableStateMachine '{self.name}' spin() - done with outcome={outcome}")
                break

            self.wait(seconds=self.sleep_duration)
        return outcome

    def get_latest_status(self):
        """
        Return the latest execution information as a BehaviorSync message.

          Note: Mirror uses derived version that cleans up mirror paths
        """
        raise NotImplementedError('Do not call this version directly - '
                                  'either OperatableStateMachine or Mirror should override')

    @classmethod
    def process_sync_request(cls):
        """Process sync request (ignored here - should be handled by derived state)."""
        Logger.localinfo('Ignoring PreemptableState process_sync_request')

    def _notify_skipped(self):
        # make sure we dont miss a preempt even if not being executed (e.g., due to priority container)
        if self._current_state is not None:
            # Prioritize handling at low level state first
            self._current_state._notify_skipped()

        if self._is_controlled and self._sub.has_msg(Topics._CMD_PREEMPT_TOPIC):
            self._sub.remove_last_msg(Topics._CMD_PREEMPT_TOPIC)
            self._pub.publish(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback(command='preempt'))
            PreemptableState.preempt = True
