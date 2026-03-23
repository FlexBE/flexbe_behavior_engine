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


"""Simplified state machine for use with FlexBE UI State machine mirror."""

from collections import deque
from threading import Event

from flexbe_core import Logger
from flexbe_core.core import PreemptableState, PreemptableStateMachine
from flexbe_core.core import State
from flexbe_core.core import StateError
from flexbe_core.core import StateMachine
from flexbe_core.core import StateMap
from flexbe_core.core import Topics
from flexbe_core.proxy.qos import QOS_OUTCOME

from flexbe_mirror.mirror_state import MirrorState

from flexbe_msgs.msg import BehaviorSync

import rclpy

from std_msgs.msg import UInt32


class MirrorStateMachine(PreemptableStateMachine):
    """Manage updates of the FlexBE mirror in response to changes."""

    _PENDING_OUTCOME_MAXLEN = 128
    # Class variable intentionally shared: nested containers (e.g. MirrorConcurrencyContainer)
    # set this flag to signal the top-level SM spin loop to run another cycle.
    _execute_flag = True

    def __init__(self, target_name, target_path, *args, **kwargs):
        """Initialize MirrorStateMachine instance."""
        super().__init__(*args, **kwargs)
        self.id = None
        self._entering = True
        self._target_name = target_name
        self._target_path = '/' + '/'.join(target_path.split('/')[1:])  # Drop top-level name
        self._cached_status_msg = None
        self._cached_status_states = None
        self._cached_status_behavior_id = BehaviorSync.INVALID
        self._pending_outcomes = deque(maxlen=self._PENDING_OUTCOME_MAXLEN)
        self._pending_outcome = None
        self._status_event_callback = None
        self._outcome_sub = None

    def _ensure_pending_outcomes(self):
        """Ensure pending outcome queue exists for unit-test stubs created via __new__."""
        if not hasattr(self, '_pending_outcomes') or self._pending_outcomes is None:
            self._pending_outcomes = deque(maxlen=self._PENDING_OUTCOME_MAXLEN)

    def _clear_pending_outcomes(self):
        """Clear pending outcomes and legacy alias."""
        self._ensure_pending_outcomes()
        self._pending_outcomes.clear()
        self._pending_outcome = None

    def _defer_outcome_mirror(self, state_id, desired_outcome, now_sec=None, reason=0):
        """Queue an outcome until its target state/container is locally ready to consume it."""
        del reason
        self._ensure_pending_outcomes()
        if self._pending_outcomes and self._pending_outcomes[-1][0] == state_id and \
                self._pending_outcomes[-1][1] == desired_outcome:
            return False
        if now_sec is None:
            now_sec = 0.0
        self._pending_outcomes.append((state_id, desired_outcome, now_sec))
        self._pending_outcome = desired_outcome
        return True

    def _pop_pending_outcome_for_state(self, state_id):
        """Pop the oldest queued outcome for a specific state ID."""
        self._ensure_pending_outcomes()
        if not self._pending_outcomes:
            self._pending_outcome = None
            return None

        kept = deque(maxlen=self._pending_outcomes.maxlen)
        popped = None
        for queued_state_id, desired_outcome, seen_at in self._pending_outcomes:
            if popped is None and queued_state_id == state_id:
                popped = desired_outcome
                continue
            kept.append((queued_state_id, desired_outcome, seen_at))

        self._pending_outcomes = kept
        self._pending_outcome = None if popped is None else popped
        return popped

    def _active_state_id_set(self):
        """Return currently active deep-state IDs."""
        try:
            deep_states = self.get_deep_states()
        except Exception:  # pylint: disable=W0703
            active_ids = set()
        else:
            active_ids = {state.state_id for state in deep_states if state is not None and hasattr(state, 'state_id')}
        active_ids.add(self._state_id)
        return active_ids

    def _promote_pending_outcome_if_relevant(self, now_sec):
        """Promote the oldest queued outcome that now belongs to the active path."""
        del now_sec
        self._ensure_pending_outcomes()
        if not self._pending_outcomes:
            return False
        if MirrorState._last_state_id is not None or MirrorState._last_state_outcome is not None:
            return False

        active_ids = self._active_state_id_set()
        kept = deque(maxlen=self._pending_outcomes.maxlen)
        promoted = None
        for state_id, desired_outcome, seen_at in self._pending_outcomes:
            if promoted is None:
                if state_id == self.state_id and self._current_state is not None:
                    kept.append((state_id, desired_outcome, seen_at))
                    continue
                if state_id in active_ids:
                    promoted = (state_id, desired_outcome)
                    continue
            kept.append((state_id, desired_outcome, seen_at))

        self._pending_outcomes = kept
        if promoted is None:
            return False

        MirrorState._last_state_id, MirrorState._last_state_outcome = promoted
        self._pending_outcome = promoted[1]
        MirrorStateMachine._execute_flag = True
        return True

    def _consume_pending_outcome_mirror(self, userdata):
        """Apply a deferred container outcome once the local child path has completed."""
        if self._current_state is not None:
            return None
        desired_outcome = self._pop_pending_outcome_for_state(self.state_id)
        if desired_outcome is None:
            return None
        return self.on_exit_mirror(userdata, desired_outcome)

    def _drain_buffered_outcomes(self, outcome_sub):
        """Drain all buffered outcome messages into the pending queue."""
        drained_any = False
        active_ids = None
        while outcome_sub.has_buffered(Topics._OUTCOME_TOPIC):
            if PreemptableState.preempt:
                break
            drained_any = True
            msg = outcome_sub.get_from_buffer(Topics._OUTCOME_TOPIC)
            if msg.data == 0:
                # Ignore legacy raw-zero barrier messages left from older peers.
                continue

            state_id, incoming_outcome = StateMap.unhash(msg.data)
            if active_ids is None:
                active_ids = self._active_state_id_set()
            if state_id in active_ids:
                self._defer_outcome_mirror(state_id, incoming_outcome)
            else:
                self._defer_outcome_mirror(state_id, incoming_outcome)

        return drained_any

    def _consume_promoted_top_level_outcome(self, userdata, start_time):
        """Consume an already promoted top-level outcome if present."""
        if MirrorState._last_state_id != self.state_id:
            return None
        if MirrorState._last_state_outcome is None:
            MirrorState._last_state_id = None
            return None

        desired_outcome = MirrorState._last_state_outcome
        if self._current_state is not None:
            Logger.localwarn(f'Mirror SM top-level spin for \'{self.name.replace("_mirror", "")}\' '
                             f'deferred premature outcome index={desired_outcome} while '
                             f'current state=\'{self._current_state.name.replace("_mirror", "")}\' '
                             f'({self._current_state.state_id}) is still active')
            self._defer_outcome_mirror(self.state_id, desired_outcome)
            MirrorState._last_state_id = None
            MirrorState._last_state_outcome = None
            return None

        if self._last_outcome is not None:
            Logger.localwarn(f'Mirror SM top-level spin for \'{self.name.replace("_mirror", "")}\' '
                             f'of \'{self.path.replace("_mirror", "")}: '
                             f'Already processed outcome={self._last_outcome!r} for '
                             f' state \'{self.name.replace("_mirror", "")}\' ({self.state_id}) given new '
                             f'outcome index={desired_outcome} - '
                             f'reprocessing anyway in thread started at {start_time.nanoseconds}')

        MirrorState._last_state_id = None  # Flag that the message was handled
        outcome = self.on_exit_mirror(userdata, desired_outcome)
        MirrorState.publish_update(self.state_id)  # Notify back at top-level before exit
        MirrorState._last_state_outcome = None  # Flag that the message was handled
        self._pending_outcome = None
        return outcome

    def _notify_status_event(self, active_states):
        """Notify the owning mirror node when the active mirrored path changes."""
        if self._status_event_callback is None:
            return
        try:
            self._status_event_callback(active_states)
        except Exception as exc:  # pylint: disable=W0703
            Logger.localwarn("MirrorStateMachine '%s' (%s) status callback failed: %s - %s",
                             self.name, self.state_id, type(exc).__name__, exc)

    def spin(self, start_time, state_map):
        """Spin the execute in loop for Mirror."""
        Logger.localinfo("Mirror: begin spinning for '%s' (%s) in thread with start time = %s ns",
                         self.name, self.id, start_time.nanoseconds)

        userdata = None  # Not used in mirror
        timing_event = Event()
        callback_inst_id = id(timing_event)

        outcome_sub = self._outcome_sub
        outcome_sub.subscribe(Topics._OUTCOME_TOPIC, UInt32,
                              callback=lambda _msg: timing_event.set(),
                              qos=QOS_OUTCOME,
                              inst_id=callback_inst_id)

        MirrorState._last_state_id = None
        MirrorState._last_state_outcome = None

        try:
            MirrorStateMachine._execute_flag = True  # Force a first pass regardless of messages
            self._last_deep_states_list = None  # Force change to send behavior update
            self._clear_pending_outcomes()
            loop_count = 0
            self._total_loop_count = 0  # Attribute only added to top-level SM
            outcome = State._preempted_name
            if self._current_state is None:
                self.on_enter_mirror(userdata)
                Logger.localinfo("Mirror: set initial state for top-level '%s' (%s) (%s) with state = %s' (%s)",
                                 self.name, self.state_id, self.id,
                                 self._current_state.name, self._current_state.state_id)

            while rclpy.ok() and not PreemptableState.preempt:
                self._total_loop_count += 1
                loop_count += 1  # For periodic updates
                try:
                    if self._drain_buffered_outcomes(outcome_sub):
                        MirrorStateMachine._execute_flag = True
                    if PreemptableState.preempt:
                        break

                    did_work = False
                    while not PreemptableState.preempt:
                        made_progress = False
                        if self._promote_pending_outcome_if_relevant(None):
                            made_progress = True

                        top_level_outcome = self._consume_promoted_top_level_outcome(userdata, start_time)
                        if top_level_outcome is not None:
                            outcome = top_level_outcome
                            did_work = True
                            break

                        if MirrorStateMachine._execute_flag or \
                                MirrorState._last_state_id is not None or \
                                MirrorState._last_state_outcome is not None:
                            MirrorStateMachine._execute_flag = False
                            pending_before = (MirrorState._last_state_id, MirrorState._last_state_outcome)
                            current_before = self._current_state
                            last_outcome_before = self._last_outcome
                            outcome = self._execute_current_state_mirror(userdata)
                            pending_after = (MirrorState._last_state_id, MirrorState._last_state_outcome)

                            if outcome is not None or pending_after != pending_before or \
                                    self._current_state is not current_before or \
                                    self._last_outcome != last_outcome_before:
                                made_progress = True

                            if MirrorState._last_state_id is not None or MirrorState._last_state_outcome is not None:
                                # This should not happen unless system is out of sync
                                Logger.logwarn(f"MirrorStateMachine '{self.name}' ({self.state_id}) spin() - "
                                               f'no state handled outcome from {MirrorState._last_state_id} '
                                               f'outcome index={MirrorState._last_state_outcome}')
                                if MirrorState._last_state_id is not None and MirrorState._last_state_outcome is not None:
                                    self._defer_outcome_mirror(MirrorState._last_state_id,
                                                               MirrorState._last_state_outcome)
                                MirrorState._last_state_id = None
                                MirrorState._last_state_outcome = None
                                made_progress = True

                            # Store the information for safely passing to heartbeat thread
                            deep_states = self.get_deep_states()
                            if deep_states != self._last_deep_states_list:
                                MirrorStateMachine._execute_flag = True  # Execute once more after any change,
                                with self._status_lock:
                                    self._last_deep_states_list = deep_states
                                self._notify_status_event(deep_states)
                                made_progress = True

                            if outcome is not None:
                                if self._current_state is None and self._last_outcome is not None:
                                    Logger.localinfo(
                                        "MirrorStateMachine '%s' (%s) spin() - consumed deferred top-level outcome = %s",
                                        self.name, self.state_id, outcome)
                                    did_work = True
                                    break

                                Logger.localinfo("MirrorStateMachine '%s' (%s) spin() - outcome = %s"
                                                 ' - wait for confirming top-level outcome message!',
                                                 self.name, self.state_id, outcome)

                        if not made_progress:
                            break
                        did_work = True

                    if self._current_state is None and self._last_outcome is not None:
                        break

                    if not did_work:
                        # Wait for the next outcome callback instead of polling continuously.
                        if loop_count > 100000:
                            loop_count = 0   # periodic spam for updates
                            Logger.localinfo("  SM spinner -'%s' (%s) - %s spins",
                                             self.name, self.id, self._total_loop_count)
                        timing_event.clear()
                        if outcome_sub.has_buffered(Topics._OUTCOME_TOPIC) or MirrorStateMachine._execute_flag:
                            continue
                        timing_event.wait(0.05)

                except Exception as exc:  # pylint: disable=W0703
                    Logger.logerr(f"  Exception in mirror spinner -'{self.state_id}' ({self.id})")
                    Logger.localerr(f'{type(exc)} - {exc}')
                    import traceback
                    Logger.localinfo(f"{traceback.format_exc().replace('%', '%%')}")
                    break

            Logger.localinfo("Mirror: done spinning for  '%s' (%s) with outcome = '%s' after %s spins"
                             ' in thread started at %s',
                             self.name, self.id, outcome, self._total_loop_count, start_time.nanoseconds)
            return outcome
        finally:
            outcome_sub.unsubscribe_topic(Topics._OUTCOME_TOPIC, inst_id=callback_inst_id)

    def destroy(self):
        """Destroy state machine."""
        Logger.localinfo(f'Destroy mirror state machine {self.name} ...')
        self._notify_stop()

    def _notify_stop(self):
        """Notify states to stop."""
        for state in self._states:
            if isinstance(state, MirrorState):
                state.on_stop()
            if isinstance(state, MirrorStateMachine):
                state._notify_stop()

    def _execute_current_state_mirror(self, userdata):
        """Define custom mirror execute method."""
        if self._current_state is None:
            # Current state might be None while waiting on final outcome message to exit SM
            return self._consume_pending_outcome_mirror(userdata)

        # Process the current state
        outcome = self._current_state.execute_mirror(userdata)
        if outcome is not None:
            MirrorStateMachine._execute_flag = True  # Spin it again
            try:
                target = self._transitions[self._current_state.name][outcome]
                self._current_state = self._labels.get(target)  # Get the new state
                if self._current_state is None:
                    # Logger.localinfo(f"SM {self.name.replace('_mirror', '')} is done, but wait for outcome message.")
                    MirrorState.publish_update(self.state_id)  # Notify back at sm-level before exit
                    return self._consume_pending_outcome_mirror(userdata)
                else:
                    # Logger.localinfo(f"SM {self.name.replace('_mirror', '')} transitioning "
                    #                  f"to '{self._current_state.name.replace('_mirror', '')}' ...")
                    self._current_state._entering = True
                    return None
            except KeyError as exc:
                err_msg = f"Returned outcome '{outcome}' is not registered as a transition from '{self._current_state}'"
                Logger.localerr(f"Mirror SM execute for '{self.name}' ({self.state_id}): {err_msg}")
                Logger.localinfo(f'  {self.name} ({self.state_id}) - labels={self._labels}')
                Logger.localinfo(f'  {self.name} ({self.state_id}) - transitions={self._transitions}')
                Logger.localinfo(f'  {self.name} ({self.state_id}) - outcomes={self._outcomes}')
                raise StateError(err_msg) from exc

        # we handle internal SM transitions using mirror outcome messages
        return None

    def execute_mirror(self, userdata):
        """Execute this SM as an internal state."""
        if self._entering:
            self.on_enter_mirror(userdata)

        if MirrorState._last_state_id == self.state_id:
            # Handle outcome of this internal SM
            # Logger.localinfo(f"Handling outcome of SM '{self.name.replace('_mirror', '')}' "
            #                  f"of '{self.path.replace('_mirror', '')}' ...")
            if self._current_state is not None:
                Logger.localwarn(f"Mirror SM execute for '{self.name}' ({self.state_id}) deferred premature "
                                 f'outcome index={MirrorState._last_state_outcome} while '
                                 f"current state='{self._current_state.name}' ({self._current_state.state_id}) is active")
                self._defer_outcome_mirror(self.state_id, MirrorState._last_state_outcome)
                MirrorState._last_state_id = None
                MirrorState._last_state_outcome = None
                return self._execute_current_state_mirror(userdata)
            if self._last_outcome is not None:
                Logger.localwarn(f"Mirror SM execute for '{self.name}' ({self.state_id}) : "
                                 f'Already processed outcome={self._last_outcome} for '
                                 f'outcome index={MirrorState._last_state_outcome} - reprocessing anyway')

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
            deep_states = [self]
            deep_states.extend(self._current_state.get_deep_states())
            return deep_states
        return [self, self._current_state] if self._current_state is not None else [self]  # Return as a list

    def get_latest_status(self):
        """Return the latest execution information as a BehaviorSync message."""
        with self._status_lock:
            active_states = self._last_deep_states_list
            behavior_id = self.id if self.id is not None else BehaviorSync.INVALID
            if self._cached_status_msg is not None and \
                    active_states is self._cached_status_states and \
                    behavior_id == self._cached_status_behavior_id:
                return self._cached_status_msg

        msg = BehaviorSync()
        msg.behavior_id = behavior_id

        if active_states is not None:
            for active in active_states:
                if active is not None:
                    outcome_index = None
                    if active._last_outcome is not None:
                        try:
                            outcome_index = active._outcomes.index(active._last_outcome)
                        except Exception:  # pylint: disable=W0703
                            Logger.localerr(f"Invalid outcome='{active._last_outcome} for '{active}' - ignore outcome!")

                    msg.current_state_checksums.append(StateMap.hash(active, outcome_index))
        else:
            Logger.localinfo(f' Mirror get_latest_status:  No active states for {msg.behavior_id}!')

        with self._status_lock:
            current_behavior_id = self.id if self.id is not None else BehaviorSync.INVALID
            if active_states is self._last_deep_states_list and behavior_id == current_behavior_id:
                self._cached_status_msg = msg
                self._cached_status_states = active_states
                self._cached_status_behavior_id = behavior_id
        return msg

    def on_enter_mirror(self, userdata):
        """Enter mirror statemachine."""
        self._entering = False
        self._last_outcome = None
        self._clear_pending_outcomes()
        self.assert_consistent_transitions()
        self._current_state = self.initial_state
        self._last_outcome = None
        self._current_state._entering = True  # force state to handle enter on first execute
        self._userdata = None  # not used in mirror
        MirrorState.publish_update(self.state_id)

    def on_exit_mirror(self, userdata, desired_outcome=-1):
        """Exit mirror statemachine."""
        try:
            self._clear_pending_outcomes()
            if self._current_state is not None:
                self._current_state._entering = True
                self._current_state.on_exit_mirror(userdata, -1)  # Preempted
            if desired_outcome != -1:
                if desired_outcome == StateMap._MAX_OUTCOME:
                    self._last_outcome = State._preempted_name
                else:
                    self._last_outcome = self.outcomes[desired_outcome]
            self._current_state = None
            self._entering = True
            MirrorState.publish_update(self.state_id + 255)  # publish that we "entered" container to exit
            return self._last_outcome
        except Exception:  # pylint: disable=W0703
            Logger.localerr(f"Error: MirrorStateMachine execute for '{self.name}': "
                            f'outcome index {desired_outcome} is not relevant ({len(self.outcomes)}) ')
            return None
