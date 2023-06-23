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


"""A state machine that can be preempted."""
import threading
import zlib

import rclpy
from std_msgs.msg import Empty
from flexbe_msgs.msg import BehaviorSync

from flexbe_core.core.lockable_state_machine import LockableStateMachine
from flexbe_core.core.preemptable_state import PreemptableState
from flexbe_core.logger import Logger
from flexbe_core.proxy import ProxySubscriberCached


class PreemptableStateMachine(LockableStateMachine):
    """
    A state machine that can be preempted.

    If preempted, the state machine will return the outcome preempted.
    """

    _preempted_name = 'preempted'

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # always listen to preempt so that the behavior can be stopped even if unsupervised
        self._preempt_topic = 'flexbe/command/preempt'
        self._sub = ProxySubscriberCached({self._preempt_topic: Empty}, inst_id=id(self))
        self._sub.set_callback(self._preempt_topic, self._preempt_cb, inst_id=id(self))
        self._status_lock = threading.Lock()
        self._last_deep_state_name = None
        self._last_deep_state_path = None

    def _preempt_cb(self, msg):
        if not self._is_controlled:
            Logger.localinfo(f'Preempting {self.name}!')
            PreemptableState.preempt = True

    @staticmethod
    def add(label, state, transitions=None, remapping=None):
        transitions[PreemptableState._preempted_name] = PreemptableStateMachine._preempted_name
        LockableStateMachine.add(label, state, transitions, remapping)

    @property
    def _valid_targets(self):
        return super()._valid_targets + [PreemptableStateMachine._preempted_name]

    def spin(self, userdata=None):
        outcome = None
        while rclpy.ok():
            outcome = self.execute(userdata)

            # Store the information for safely passing to heartbeat thread
            deep_state = self.get_deep_state()
            if deep_state:
                if deep_state.name != self._last_deep_state_name:
                    with self._status_lock:
                        self._last_deep_state_path = str(deep_state.path)
                        self._last_deep_state_name = str(deep_state.name)
            else:
                if self._last_deep_state_name is not None:
                    with self._status_lock:
                        self._last_deep_state_path = None
                        self._last_deep_state_name = None

            if self._inner_sync_request:
                # Top-level state machine with sync request
                self.process_sync_request()

            if outcome is not None:
                Logger.loginfo(f"PreemptableStateMachine {self.name} spin() - done with outcome={outcome}")
                break

            self.sleep()

        return outcome

    def get_latest_status(self):
        """
        Return the latest execution information as a BehaviorSync message.

          Note: Mirror uses derived version that cleans up mirror paths
        """
        with self._status_lock:
            path = self._last_deep_state_path

        msg = BehaviorSync()
        msg.behavior_id = -1
        if path is not None:
            msg.current_state_checksum = zlib.adler32(path.encode()) & 0x7fffffff
        else:
            msg.current_state_checksum = -1
        return msg

    @classmethod
    def process_sync_request(cls):
        Logger.localinfo("Ignoring PreemptableState process_sync_request")
