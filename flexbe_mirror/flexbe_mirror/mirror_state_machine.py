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

import rclpy

from std_msgs.msg import UInt32

from flexbe_core import Logger
from flexbe_core.core import PreemptableState, PreemptableStateMachine
from flexbe_core.core import StateMachine
from flexbe_core.core import StateMap
from flexbe_core.core.exceptions import StateError
from flexbe_core.core.topics import Topics
from flexbe_core.proxy import ProxySubscriberCached

from flexbe_msgs.msg import BehaviorSync
from flexbe_mirror.mirror_state import MirrorState


class MirrorStateMachine(PreemptableStateMachine):
    """Manage updates of the FlexBE mirror in response to changes."""

    _execute_flag = True  # on change, we should execute cycle

    def __init__(self, target_name, target_path, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.id = None
        self._entering = True
        self._target_name = target_name
        self._target_path = "/" + "/".join(target_path.split("/")[1:])  # Drop top-level name

    def spin(self, start_time, userdata=None):
        """Spin the execute in loop for Mirror."""
        Logger.localinfo(f"Mirror: begin spinning for '{self.name}' ({self.id}) "
                         f" in thread with start time = {start_time.nanoseconds} ns")

        timing_event = Event()

        # Only the top-level SM needs the outcome topic subscribed by mirror
        outcome_sub = ProxySubscriberCached()

        MirrorState._last_state_id = None
        MirrorState._last_state_outcome = None

        MirrorStateMachine._execute_flag = True  # Force a first pass regardless of messages
        loop_count = 0
        self._total_loop_count = 0  # Attribute only added to top-level SM
        outcome = PreemptableState._preempted_name
        if self._current_state is None:
            self.on_enter_mirror(userdata)
            Logger.localinfo(f"Mirror: set initial state for top-level '{self.name}' ({self.state_id}) ({self.id}) "
                             f" with state = {self._current_state.name}' ({self._current_state._state_id})")

        while rclpy.ok() and not PreemptableState.preempt:
            self._total_loop_count += 1
            loop_count += 1  # For periodic updates
            try:
                # Grab the latest outcome
                if outcome_sub.has_buffered(Topics._OUTCOME_TOPIC):
                    # Mirror states only change in response to an outcome message.
                    # We will process every message to ensure consistency
                    msg = outcome_sub.get_from_buffer(Topics._OUTCOME_TOPIC)
                    state_id, outcome = StateMap.unhash(msg.data)
                    # Logger.localinfo(f"  outcome {state_id} {outcome} in thread started at {start_time.nanoseconds}")

                    # Store data for handling by execute function in appropriate state
                    MirrorState._last_state_id = state_id
                    MirrorState._last_state_outcome = outcome
                    if MirrorState._last_state_id == self.state_id:
                        # Handle this top-level outcome
                        if self._last_outcome is not None:
                            Logger.localwarn(f"Mirror SM top-level spin for {self.name} : "
                                             f"Already processed outcome={self._last_outcome} for "
                                             f" state '{self.name}' ({self.state_id}) given new "
                                             f"outcome index={MirrorState._last_state_outcome} - "
                                             f"reprocessing anyway in thread started at {start_time.nanoseconds}")

                        MirrorState._last_state_id = None  # Flag that the message was handled
                        if MirrorState._last_state_outcome is not None:
                            outcome = self.on_exit_mirror(userdata, MirrorState._last_state_outcome)
                            MirrorState.publish_update(self._target_path)  # Notify back at top-level before exit
                            MirrorState._last_state_outcome = None  # Flag that the message was handled
                            Logger.localinfo(f" top-level outcome {outcome} for {state_id} "
                                             f"in thread started at {start_time.nanoseconds}")
                            break  # Outcome at the top-level

                    # Some change to process
                    MirrorStateMachine._execute_flag = True

                # No top-level outcome, so execute the active states looking for outcomes
                if MirrorStateMachine._execute_flag:
                    MirrorStateMachine._execute_flag = False
                    # Process the latest outcome by active states
                    outcome = self._execute_current_state_mirror(userdata)

                    if MirrorState._last_state_id is not None or MirrorState._last_state_outcome is not None:
                        # This should not happen unless system is out of sync
                        Logger.logwarn(f"MirrorStateMachine '{self.name}' ({self._state_id}) spin() - "
                                       f"no state handled outcome from {MirrorState._last_state_id} "
                                       f"outcome index={MirrorState._last_state_outcome}")

                    # Store the information for safely passing to heartbeat thread
                    deep_states = self.get_deep_states()
                    if deep_states != self._last_deep_states_list:
                        MirrorStateMachine._execute_flag = True  # Execute once more after any change,
                        with self._status_lock:
                            self._last_deep_states_list = deep_states

                        # In case of internal return in concurrency container send another update to UI
                        # for the deepest active state
                        if len(deep_states) > 0:
                            MirrorState.publish_update(deep_states[-1]._target_path)

                    if outcome is not None:
                        Logger.localinfo(f"MirrorStateMachine '{self.name}' ({self._state_id}) spin() - outcome = {outcome}"
                                         f" - wait for confirming top-level outcome message!")

                else:
                    # Process fast independent of simulation time in order to keep up with onboard
                    if loop_count > 50000:
                        loop_count = 0   # periodic spam for updates
                        Logger.localinfo(f"  SM spinner -'{self.name}' ({self.id}) - "
                                         f"after {self._total_loop_count} spins in thread started at {start_time.nanoseconds}")
                    timing_event.wait(0.0002)  # minor wait for next message if we didn't process anything previous loop

            except Exception as exc:  # pylint: disable=W0703
                Logger.logerr(f"  Exception in mirror spinner -'{self.state_id}' ({self.id})")
                Logger.localerr(f"{type(exc)} - {exc}")
                import traceback
                Logger.localinfo(f"{traceback.format_exc().replace('%', '%%')}")
                break

        Logger.localinfo(f"Mirror: done spinning for  '{self.name}' ({self.id}) with outcome = '{outcome}' "
                         f" after {self._total_loop_count} spins"
                         f" in thread started at {start_time.nanoseconds}")
        return outcome

    def destroy(self):
        Logger.localinfo(f'Destroy mirror state machine {self.name} ...')
        self._notify_stop()

    def _notify_stop(self):
        for state in self._states:
            if isinstance(state, MirrorState):
                state.on_stop()
            if isinstance(state, MirrorStateMachine):
                state._notify_stop()

    def _execute_current_state_mirror(self, userdata):
        """Define custom mirror execute method."""
        if self._current_state is None:
            # Current state might be None while waiting on final outcome message to exit SM
            return None

        # Process the current state
        outcome = self._current_state.execute_mirror(userdata)
        if outcome is not None:
            MirrorStateMachine._execute_flag = True  # Spin it again
            try:
                target = self._transitions[self._current_state.name][outcome]
                self._current_state = self._labels.get(target)  # Get the new state
                if self._current_state is None:
                    return target
                else:
                    self._current_state._entering = True
                    return None
            except KeyError as exc:
                err_msg = f"Returned outcome '{outcome}' is not registered as a transition from '{self._current_state}'"
                Logger.localerr(f"Mirror SM execute for '{self.name}' ({self._state_id}): {err_msg}")
                Logger.localinfo(f"  {self.name} ({self._state_id}) - labels={self._labels}")
                Logger.localinfo(f"  {self.name} ({self._state_id}) - transitions={self._transitions}")
                Logger.localinfo(f"  {self.name} ({self._state_id}) - outcomes={self._outcomes}")
                raise StateError(err_msg) from exc

        # we handle internal SM transitions using mirror outcome messages
        return None

    def execute_mirror(self, userdata):
        """Execute this SM as an internal state."""
        if self._entering:
            self.on_enter_mirror(userdata)

        if MirrorState._last_state_id == self.state_id:
            # Handle outcome of this internal SM
            if self._last_outcome is not None:
                Logger.localwarn(f"Mirror SM execute for '{self.name}' ({self._state_id}) : "
                                 f"Already processed outcome={self._last_outcome} for "
                                 f"outcome index={MirrorState._last_state_outcome} - reprocessing anyway")

            MirrorState._last_state_id = None  # Flag that the message was handled
            if MirrorState._last_state_outcome is not None:
                desired_outcome = MirrorState._last_state_outcome
                MirrorState._last_state_outcome = None
                return self.on_exit_mirror(userdata, desired_outcome)

        return self._execute_current_state_mirror(userdata)

    def get_deep_states(self):
        """
        Recursively look for the currently executing states.

        Traverse all state machines down to the terminal child state that is not a container.
        (Except concurrency containers, which override this method)

        @return: The list of active states (not state machine)
        """
        if isinstance(self._current_state, StateMachine):
            return self._current_state.get_deep_states()
        return [self._current_state] if self._current_state is not None else []  # Return as a list

    def get_latest_status(self):
        """Return the latest execution information as a BehaviorSync message."""
        msg = BehaviorSync()
        with self._status_lock:
            active_states = self._last_deep_states_list
            msg.behavior_id = self.id if self.id is not None else BehaviorSync.INVALID

        if active_states is not None:
            for active in active_states:
                if active is not None:
                    outcome_index = 0
                    if active._last_outcome is not None:
                        try:
                            outcome_index = active._outcomes.index(active._last_outcome)
                        except Exception:  # pylint: disable=W0703
                            Logger.localerr(f"Invalid outcome='{active._last_outcome} for '{active}' - ignore outcome!")

                    msg.current_state_checksums.append(StateMap.hash(active, outcome_index))
        else:
            Logger.localinfo(f" Mirror get_latest_status:  No active states for {msg.behavior_id}!")
        return msg

    def on_enter_mirror(self, userdata):
        self._entering = False
        self._last_outcome = None
        self.assert_consistent_transitions()
        self._current_state = self.initial_state
        self._userdata = None  # not used in mirror
        MirrorState.publish_update(self._target_path)

    def on_exit_mirror(self, userdata, desired_outcome=-1):
        try:
            if self._current_state is not None:
                self._current_state._entering = True
                self._current_state.on_exit(userdata, -1)  # Preempted
            self._last_outcome = self.outcomes[desired_outcome]
            self._current_state = None
            self._entering = True
            return self._last_outcome
        except Exception:  # pylint: disable=W0703
            Logger.localerr(f"Error: MirrorStateMachine execute for {self.name}: "
                            f"outcome index {desired_outcome} is not relevant ({len(self.outcomes)}) ")
            return None
