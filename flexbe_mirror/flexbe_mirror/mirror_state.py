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


"""Simplified state for use with FlexBE UI State machine mirror."""

from std_msgs.msg import String

from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.core.preemptable_state import PreemptableState
from flexbe_core.core.topics import Topics
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached


class MirrorState(EventState):
    """
    Simplified state for use with FlexBE UI State machine mirror.

    This state will display its possible outcomes as buttons in the GUI
    and is designed in a way to be created dynamically.
    """

    # Hold data from last outcome message being processed by mirror
    _last_state_id = None
    _last_state_outcome = None
    _last_target_path = None

    _pub = None

    def __init__(self, target_name, target_path, given_outcomes, outcome_autonomy):
        # pylint: disable=unused-argument
        super().__init__(outcomes=given_outcomes)
        self._outcomes.append(PreemptableState._preempted_name)  # Add preempted to outcomes list (for -1 outcome)
        self._target_name = target_name
        self._target_path = "/" + "/".join(target_path.split("/")[1:])  # Drop top-level name
        MirrorState._last_target_path = None  # reset any time that we build a new state machine

        if MirrorState._pub is None:
            # Allow access to standard proxies initialied by flexbe_mirror
            MirrorState._pub = ProxyPublisher()

    @classmethod
    def publish_update(cls, target_path):
        """Publish latest deep state for UI control."""
        if target_path != MirrorState._last_target_path:
            MirrorState._last_target_path = target_path
            MirrorState._pub.publish(Topics._BEHAVIOR_UPDATE_TOPIC, String(data=target_path))

    def execute_mirror(self, userdata):
        if self._entering:
            self.on_enter_mirror(userdata)

        if MirrorState._last_state_id == self.state_id:
            # Only process if relevant to this state
            if self._last_outcome is not None:
                Logger.localwarn(f"Already processed outcome={self._last_outcome} for "
                                 f" state '{self.name}' ({self.state_id}) given new "
                                 f"outcome index={MirrorState._last_state_outcome} - reprocessing anyway")

            MirrorState._last_state_id = None  # Flag that the message was handled
            if MirrorState._last_state_outcome is not None:
                desired_outcome = MirrorState._last_state_outcome
                MirrorState._last_state_outcome = None  # Flag that the message was handled
                return self.on_exit_mirror(userdata, desired_outcome)
        return None

    def on_enter_mirror(self, userdata):
        self._entering = False
        self._last_outcome = None
        MirrorState.publish_update(self._target_path)

    def on_exit_mirror(self, userdata, desired_outcome):
        try:
            self._last_outcome = self.outcomes[desired_outcome]
            return self._last_outcome
        except Exception as exc:  # pylint: disable=W0703
            Logger.localerr(f"Error: MirrorState execute for {self.name}: "
                            f"outcome index {desired_outcome} is not relevant ({len(self.outcomes)}) ")
            import traceback
            Logger.localerr(f"{type(exc)} - {exc}\n{traceback.format_exc().replace('%', '%%')}")
            return None
