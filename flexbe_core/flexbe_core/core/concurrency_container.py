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
from flexbe_core.core.event_state import EventState
from flexbe_core.core.exceptions import StateError, StateMachineError, UserDataError
from flexbe_core.core.lockable_state_machine import LockableStateMachine
from flexbe_core.core.operatable_state_machine import OperatableStateMachine
from flexbe_core.core.preemptable_state import PreemptableState
from flexbe_core.core.priority_container import PriorityContainer
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
        self._type = OperatableStateMachine.ContainerType.ConcurrencyContainer.value
        self._manual_transition_requested = None

    @property
    def sleep_duration(self):
        """Sleep duration in seconds."""
        sleep_dur = float('inf')
        for state in self._states:
            sleep_dur = min(sleep_dur, state.sleep_duration)

        return sleep_dur

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
            assert state in self._states, "get required autonomy in ConcurrencyContainer - state doesn't match!"
            return self._autonomy[state.name][outcome]
        except Exception as exc:
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
        if self._is_controlled and self._sub.has_buffered(Topics._CMD_TRANSITION_TOPIC):
            # Special handling in concurrency container - can be either ConcurrencyContainer or one of several internal states.
            command_msg = self._sub.peek_at_buffer(Topics._CMD_TRANSITION_TOPIC)

            if command_msg.target == self.state_id:
                cmd_msg2 = self._sub.get_from_buffer(Topics._CMD_TRANSITION_TOPIC)  # Using here, so clear from buffer
                assert cmd_msg2 is command_msg, 'Unexpected change in CMD_TRANSITION_TOPIC buffer'
                Logger.localinfo(f"ConcurrencyContainer '{self.path}' is handling the transition cmd msg={command_msg}")

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
                Logger.localinfo(f"\x1b[94mConcurrencyContainer '{self.name}' - storing {command_msg} transition request\x1b[0m")
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
                    # @TODO - Should we be using path not name here?
                    command_msg = self._manual_transition_requested
                    cmd_msg2 = self._sub.get_from_buffer(Topics._CMD_TRANSITION_TOPIC)  # Using here, so clear from buffer
                    assert cmd_msg2 is command_msg, 'Something is up with handling of buffer for CMD_TRANSITION_TOPIC'
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

                        # ConcurrencyContainer bypasses normal operatable state handling of manual request, so do that here
                        state._publish_outcome(outcome)

                        self._pub.publish(Topics._CMD_FEEDBACK_TOPIC,
                                          CommandFeedback(command='transition',
                                                          args=[f'{command_msg.target}', f'{state.state_id}']))
                        Logger.localerr(f'--> Manually triggered outcome {outcome} ({command_msg.outcome}) '
                                        f"of state '{state.name}' from inside ConcurrencyContainer '{self.name}'")
                        continue
                    else:
                        Logger.localerr(f"--> Invalid outcome {command_msg.outcome} request for state '{state.name}' "
                                        f"from inside concurrency '{self.name}'\n{state.outcomes}")

            if (PriorityContainer.active_container is not None
                and not all(a == s for a, s in zip(PriorityContainer.active_container.split('/'),
                                                   state.path.split('/')))):
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

            if state.sleep_duration <= 0 or self._manual_transition_requested is not None:  # ready to execute
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
            # catch any exception and log here, but re-raise to preempt behavior
            result = None
            self._last_exception = exc
            Logger.logerr('ConcurrencyContainer: Failed to execute state %s:\n%s' % (self.current_state_label, str(exc)))
            import traceback  # pylint: disable=C0415
            Logger.localinfo(traceback.format_exc().replace('%', '%%'))
            if isinstance(exc, (StateError, StateMachineError, UserDataError)):
                self._last_exception = exc
            else:
                self._last_exception = StateError(str(exc))
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

    def on_exit(self, userdata, states=None):
        """Call when concurrency container exits."""
        Logger.localinfo(f"ConcurrencyContainer on_exit for '{self}'.")
        for state in self._states if states is None else states:
            if state.name in self._returned_outcomes and self._returned_outcomes[state.name] is not None:
                if not state._exited:
                    Logger.localinfo(f"\x1b[93mCC '{self.name}'  - '{state.name}' is in returned outcomes "
                                     f"w/ '{self._returned_outcomes[state.name] }' but has not exited!\x1b[0m")
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
        Recursively look for the currently executing states.

        Traverse all state machines down to the terminal child state that is not a container.

        @return: The list of active states (not state machine)
        """
        deep_states = [self]  # Concurrency acts as both state and container for this purpose
        for state in self._states:
            # Internal states (after skipping concurrency container self)
            if isinstance(state, LockableStateMachine):
                deep_states.extend(state.get_deep_states())
            else:
                deep_states.append(state)
        return deep_states

    def _notify_skipped(self):
        # make sure we dont miss a preempt even if not being executed (e.g., due to priority container)
        for state in self._current_state:
            # Prioritize handling at low level state first
            state._notify_skipped()

        if self._is_controlled and self._sub.has_msg(Topics._CMD_PREEMPT_TOPIC):
            self._sub.remove_last_msg(Topics._CMD_PREEMPT_TOPIC)
            self._pub.publish(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback(command='preempt'))
            PreemptableState.preempt = True
