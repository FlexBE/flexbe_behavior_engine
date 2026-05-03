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


"""
A state machine that can be operated.

It synchronizes its current state with the mirror and supports some control mechanisms.
"""
from sys import maxsize as MAX_SIZE

from flexbe_core.core.event_state import EventState
from flexbe_core.core.exceptions import StateError, StateMachineError, UserDataError
from flexbe_core.core.lockable_state_machine import LockableStateMachine
from flexbe_core.core.operatable_state_machine import OperatableStateMachine
from flexbe_core.core.preemptable_state import PreemptableState
from flexbe_core.core.priority_container import PriorityContainer
from flexbe_core.core.ros_state import RosState
from flexbe_core.core.state import State
from flexbe_core.core.topics import Topics
from flexbe_core.core.user_data import UserData
from flexbe_core.logger import Logger
from flexbe_core.state_logger import StateLogger

from flexbe_msgs.msg import CommandFeedback, OutcomeRequest


class ConcurrencyContainer(OperatableStateMachine):
    """
    A state machine that can be operated.

    It synchronizes its current state with the mirror and supports some control mechanisms.
    """

    def __init__(self, conditions=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._conditions = conditions if conditions else {}
        self._returned_outcomes = {}
        self._current_state = None
        self._deep_states_cache_active_states = None
        self._type = OperatableStateMachine.ContainerType.ConcurrencyContainer.value
        self._manual_transition_requested = None

    @property
    def target_wakeup_ns(self):
        """Return the earliest absolute wakeup time across active child states."""
        target_wakeup_ns = MAX_SIZE  # Absurdly large for our use
        for state in self._states:
            target_wakeup_ns = min(target_wakeup_ns, state.target_wakeup_ns)

        return target_wakeup_ns

    @property
    def current_state(self):
        """Return current state of Concurrency container, which is itself and list of active states."""
        return self

    @property
    def current_state_label(self):
        """Return current state name of Concurrency container. which is itself."""
        return self.name

    def get_required_autonomy(self, outcome, state):
        """Return required autonomy level for this outcome."""
        try:
            if state not in self._states:
                raise StateError(f"get_required_autonomy: state '{state.name}' is not in ConcurrencyContainer '{self.name}'")
            return self._autonomy[state.name][outcome]
        except (StateError, AttributeError, KeyError, TypeError) as exc:
            Logger.error(f"Failure to retrieve autonomy for '{self.name}' in ConcurrencyContainer - "
                         f"  current state label='{self.name}' state='{state.name}' outcome='{outcome}'.")
            Logger.localerr(f'error={type(exc)} - {exc}')
            Logger.localerr(f'current_state={self._current_state}')
            Logger.localerr(f'autonomy={self._autonomy}')

    def _execute_current_state(self):
        """Execute the current states within this concurrency container."""
        # execute all states that are done with sleeping and determine next sleep duration
        self._inner_sync_request = False  # clear prior request for lower level state
        self._current_state = []  # Concurrency container has multiple active states so use list

        self._manual_transition_requested = None
        if self._is_controlled:
            # Special handling in concurrency container - can be either ConcurrencyContainer or one of several internal states.
            command_msg = self._sub.peek_if_buffered(Topics._CMD_TRANSITION_TOPIC)

            if command_msg is not None and command_msg.target == self.state_id:
                cmd_msg2 = self._sub.get_from_buffer(Topics._CMD_TRANSITION_TOPIC)  # Using here, so clear from buffer
                if cmd_msg2 is not command_msg:
                    Logger.localerr(f"ConcurrencyContainer '{self.path}': unexpected change in CMD_TRANSITION_TOPIC buffer "
                                    f'(expected {command_msg}, got {cmd_msg2}) - dropping transition.')
                    return None
                Logger.localinfo(f"ConcurrencyContainer '{self.path}' is handling the transition cmd msg={command_msg}")

                if not 0 <= command_msg.outcome < len(self.outcomes):
                    self._pub.publish(Topics._CMD_FEEDBACK_TOPIC,
                                      CommandFeedback(command='transition', args=['invalid', f'{command_msg.target}']))
                    Logger.localerr(f"--> Invalid outcome {command_msg.outcome} request for concurrency container '{self.name}'")
                    return None

                self._force_transition = True
                outcome = self.outcomes[command_msg.outcome]
                self._manual_transition_requested = outcome
                self._pub.publish(Topics._CMD_FEEDBACK_TOPIC,
                                  CommandFeedback(command='transition',
                                                  args=[f'{command_msg.target}', f'{self.state_id}']))  # string[]
                Logger.localwarn(f"--> Manually triggered outcome {outcome} of concurrency container '{self.path}'")
                self._publish_outcome(outcome)

                self._returned_outcomes = {}
                self._current_state = None
                self._last_outcome = outcome
                return outcome
            else:
                if command_msg is not None:
                    Logger.localinfo(f"\x1b[94mConcurrencyContainer '{self.name}' - "
                                     f'storing {command_msg} transition request\x1b[0m')
                self._manual_transition_requested = command_msg

        if self._is_controlled and self._last_requested_outcome is not None:
            # We have already processed the current state and received an outcome
            # We are waiting on outcome confirmation from the OCS
            Logger.loginfo_throttle(2.0, f"CC '{self.path}' is waiting on user to confirm outcome")
            return None

        for state in self._states:
            if state.name in self._returned_outcomes and self._returned_outcomes[state.name] is not None:
                # print(f"   in current {self._name} : state '{state.name}' is already done.", flush=True)
                continue  # already done with executing

            if self._manual_transition_requested is not None:
                if self._manual_transition_requested.target == state.state_id:
                    # Transition request applies to this state
                    # Use state label keys (state.name) consistently in this container.
                    # Labels are unique per container and align with _remappings/_conditions.
                    command_msg = self._manual_transition_requested
                    cmd_msg2 = self._sub.get_from_buffer(Topics._CMD_TRANSITION_TOPIC)  # Using here, so clear from buffer
                    if cmd_msg2 is not command_msg:
                        msg = (f"ConcurrencyContainer '{self.name}': unexpected change in CMD_TRANSITION_TOPIC buffer "
                               f"for state '{state.path}' (expected {command_msg}, got {cmd_msg2}) - dropping transition.")
                        Logger.localerr(msg)
                        self._manual_transition_requested = None
                        continue
                    Logger.localinfo(f"ConcurrencyContainer '{self.name}' state '{state.path}' is handling "
                                     f"the cmd msg='{command_msg}'")
                    self._manual_transition_requested = None  # Reset at this level

                    if 0 <= command_msg.outcome < len(state.outcomes):
                        state._force_transition = True
                        outcome = state.outcomes[command_msg.outcome]
                        state._manual_transition_requested = outcome
                        self._returned_outcomes[state.name] = outcome
                        with UserData(reference=self._userdata, remap=self._remappings[state.name],
                                      input_keys=state.input_keys, output_keys=state.output_keys) as userdata:
                            Logger.localinfo(f"ConcurrencyContainer '{self}' manual transition"
                                             f" '{outcome}' and on exit for '{state}'")
                            state.on_exit(userdata)
                        state._exited = True
                        state._entering = True
                        state._last_outcome = outcome

                        # ConcurrencyContainer bypasses normal operatable state handling of manual request, so do that here
                        state._publish_outcome(outcome)

                        self._pub.publish(Topics._CMD_FEEDBACK_TOPIC,
                                          CommandFeedback(command='transition',
                                                          args=[f'{command_msg.target}', f'{state.state_id}']))
                        Logger.localwarn(f'--> Manually triggered outcome {outcome} ({command_msg.outcome}) '
                                         f"of state '{state.name}' from inside ConcurrencyContainer '{self.name}'")
                        continue
                    else:
                        Logger.localerr(f"--> Invalid outcome {command_msg.outcome} request for state '{state.name}' "
                                        f"from inside concurrency '{self.name}'\n{state.outcomes}")

            active_segments = PriorityContainer.active_container_segments
            if (
                active_segments is not None
                and not all(a == s for a, s in zip(active_segments, state.path_segments))
            ):
                if isinstance(state, EventState):
                    # Base state not a container
                    state._notify_skipped()
                    continue  # other state has priority

                # this state must be a container
                deep_states = state.get_deep_states()
                if deep_states is not None:
                    for dpst in deep_states:
                        dpst._notify_skipped()

                continue  # other state has priority

            now_ns = RosState._current_execution_time_ns
            if state.target_wakeup_ns <= now_ns or self._manual_transition_requested is not None:  # ready to execute
                # Execute if we have a pending manual transition command or state tic rate elapsed
                out = self._execute_single_state(state)
                self._returned_outcomes[state.name] = out

            # Track any state that remains as being currently active
            self._current_state.append(state)

            # we want to pass sync requests back up to parent,
            self._inner_sync_request = self._inner_sync_request or state._inner_sync_request

        # Determine concurrency outcome
        outcome = None
        if any(self._returned_outcomes[state.name] == State._preempted_name
               for state in self._states if state.name in self._returned_outcomes):
            return State._preempted_name  # handle preemption if required
        # check conditions
        for item in self._conditions:
            (out, cond) = item
            if all(sn in self._returned_outcomes and self._returned_outcomes[sn] == o for sn, o in cond):
                outcome = out
                break

        if outcome is None:
            return None

        self._current_state = None

        if self._is_controlled:
            # request outcome because autonomy level is too low
            if (not self._force_transition and self.parent is not None
                and (not self.parent.is_transition_allowed(self.name, outcome)
                     or outcome is not None and self.is_breakpoint)):
                if outcome != self._last_requested_outcome:
                    self._pub.publish(Topics._OUTCOME_REQUEST_TOPIC,
                                      OutcomeRequest(outcome=self.outcomes.index(outcome),
                                                     target=self.state_id))
                    Logger.localinfo('<-- Want result: %s > %s' % (self.path, outcome))
                    StateLogger.log('flexbe.operator', self, type='request', request=outcome,
                                    autonomy=self.parent.autonomy_level,
                                    required=self.parent.get_required_autonomy(outcome, self))
                    self._last_requested_outcome = outcome
                outcome = None
            elif outcome is not None and outcome in self.outcomes:
                # autonomy level is high enough, report the executed transition
                self._force_transition = False

        self._last_outcome = outcome
        return outcome

    def _execute_single_state(self, state, force_exit=False):
        """Execute the next state in concurrent container."""
        result = None
        try:
            with UserData(reference=self._userdata, remap=self._remappings[state.name],
                          input_keys=state.input_keys, output_keys=state.output_keys) as userdata:
                state._inner_sync_request = False  # clear any prior sync on call to individual state
                if force_exit:
                    if state._exited:
                        Logger.localinfo(f"force exit for '{state.name}' ({state.path}) but already exited?")
                    state.on_exit(userdata)
                    state._entering = True
                    state._exited = True
                    if state._last_outcome is None:
                        Logger.localinfo(f"preempting '{state.name}' ({state.path})")
                        state._last_outcome = State._preempted_name
                        state._publish_outcome(State._preempted_name)  # Normally by EventState or StateMachine.execute
                else:
                    result = state.execute(userdata)  # This is call on_exit if necessary
        except Exception as exc:  # pylint: disable=W0703
            result = None
            wrapped = exc if isinstance(exc, (StateError, StateMachineError, UserDataError)) else StateError(str(exc))
            self._last_exception = wrapped
            Logger.logerr('ConcurrencyContainer: Failed to execute state %s:\n%s' % (self.current_state_label, str(exc)))
            import traceback  # pylint: disable=C0415
            Logger.localinfo(traceback.format_exc().replace('%', '%%'))
            raise self._last_exception
        return result

    def on_enter(self, userdata):  # pylint: disable=W0613
        """Call on entering the concurrency container."""
        super().on_enter(userdata)
        self._returned_outcomes = {}
        for state in self._states:
            # Force on_enter at state level (userdata passed by _execute_single_state)
            state._entering = True  # force state to handle enter on first execute
            state._last_execution = None
            state._last_execution_ns = None

    def on_exit(self, userdata, states=None):
        """Call when concurrency container exits."""
        Logger.localinfo(f"ConcurrencyContainer on_exit for '{self}'.")
        for state in self._states if states is None else states:
            if state.name in self._returned_outcomes and self._returned_outcomes[state.name] is not None:
                if not state._exited:
                    Logger.localinfo(f"\x1b[93mCC '{self.name}'  - '{state.name}' is in returned outcomes "
                                     f"w/ '{self._returned_outcomes[state.name]}' but has not exited!\x1b[0m")
                continue  # skip states that already exited themselves
            self._execute_single_state(state, force_exit=True)

        self._current_state = None
        self._returned_outcomes = {}
        self._entering = True

        if self._last_outcome is None:
            # Publish outcome is normally invoked by StateMachine.execute
            # If no outcome set, then notify that we preempted this state
            self._publish_outcome(State._preempted_name)

        if self._last_requested_outcome is not None:
            # Logger.localinfo(f"CC '{self.name}' of '{self.path}' clear prior LRO='{self._last_requested_outcome}'.")
            self._pub.publish(Topics._OUTCOME_REQUEST_TOPIC, OutcomeRequest(outcome=255, target=self.state_id))
            self._last_requested_outcome = None

        self._exited = True
        self._entering = True  # for next entry

    def get_deep_states(self):
        """
        Return the currently active execution paths for this concurrency container.

        The returned tuple starts with this concurrency container and then
        includes the active path for each currently active child branch,
        including nested containers.

        @return: Tuple of active states and containers across active branches.
        """
        active_states = self._current_state if isinstance(self._current_state, list) else []
        active_key = tuple(active_states)

        if (self._deep_states_list_cache is not None
                and self._deep_states_cache_active_states == active_key):
            return self._deep_states_list_cache

        deep_states = [self]  # Concurrency acts as both state and container for this purpose
        for state in active_states:
            # Internal states (after skipping concurrency container self)
            if isinstance(state, LockableStateMachine):
                deep_states.extend(state.get_deep_states())
            else:
                deep_states.append(state)

        self._deep_states_list_cache = tuple(deep_states)
        self._deep_states_cache_active_states = active_key
        return self._deep_states_list_cache

    def _notify_skipped(self):
        # make sure we dont miss a preempt even if not being executed (e.g., due to priority container)
        if self._current_state is not None:
            for state in self._current_state:
                # Prioritize handling at low level state first
                state._notify_skipped()

        if self._is_controlled and self._sub.has_msg(Topics._CMD_PREEMPT_TOPIC):
            self._sub.remove_last_msg(Topics._CMD_PREEMPT_TOPIC)
            self._pub.publish(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback(command='preempt'))
            PreemptableState.preempt = True
