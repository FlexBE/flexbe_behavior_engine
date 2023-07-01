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


"""Simplified state machine for use with FlexBE UI State machine mirror."""

from threading import Event
import zlib

import rclpy

from flexbe_core import Logger
from flexbe_core.core import PreemptableStateMachine
from flexbe_msgs.msg import BehaviorSync


class MirrorStateMachine(PreemptableStateMachine):
    """Manage updates of the FlexBE mirror in response to changes."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._timing_event = Event()

    def spin(self, userdata=None):
        """Spin the execute in loop for Mirror."""
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

            if outcome is not None:
                Logger.localinfo(f"MirrorStateMachine {self.name} spin() - done with outcome={outcome}")
                break

            # Process fast independent of simulation time in order to keep up with onboard
            self._timing_event.wait(0.001)

        return outcome

    def get_latest_status(self):
        """
        Return the latest execution information as a BehaviorSync message.

        This version is typically called by the OCS mirror, so we do
        some extra cleanup.
        """
        with self._status_lock:
            path = self._last_deep_state_path

        msg = BehaviorSync()
        msg.behavior_id = -1
        if path is not None:
            path_clean = path.replace("_mirror", "")  # Ignore mirror decoration for comparison with onboard
            msg.current_state_checksum = zlib.adler32(path_clean.encode()) & 0x7fffffff
        else:
            msg.current_state_checksum = -1
        return msg
