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


"""
A state machine that can be operated.

It synchronizes its current state with the mirror and supports some control mechanisms.
"""
from flexbe_core.core.event_state import EventState
from flexbe_core.core.lockable_state_machine import LockableStateMachine
from flexbe_core.core.operatable_state import OperatableState
from flexbe_core.core.operatable_state_machine import OperatableStateMachine
from flexbe_core.core.priority_container import PriorityContainer
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
        sleep_dur = float("inf")
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
        try:
            assert state in self._current_state, "get required autonomy in ConcurrencyContainer - state doesn't match!"
            return self._autonomy[state.name][outcome]
        except Exception as exc:
            Logger.error(f"Failure to retrieve autonomy for '{self.name}' in CC - "
                         f"  current state label='{self.name}' state='{state.name}' outcome='{outcome}'.")
            Logger.localerr(f"{type(exc)} - {exc}\n\n {self._current_state}")
            Logger.localerr(f"{self._autonomy}")

    def _execute_current_state(self):
        # execute all states that are done with sleeping and determine next sleep duration
        self._inner_sync_request = False  # clear prior request for lower level state
        self._current_state = []  # Concurrency container has multiple active states so use list

        self._manual_transition_requested = None
        # Logger.localinfo(f"-concurrency container {self.name} is_controlled={self._is_controlled}"
        #                  f"  has transition request={self._sub.has_buffered(Topics._CMD_TRANSITION_TOPIC)}")
        if self._is_controlled and self._sub.has_buffered(Topics._CMD_TRANSITION_TOPIC):
            # Special handling in concurrency container - can be either CC or one of several internal states.
            command_msg = self._sub.get_from_buffer(Topics._CMD_TRANSITION_TOPIC)

            if command_msg.target == self.name:
                self._force_transition = True
                outcome = self.outcomes[command_msg.outcome]
                self._manual_transition_requested = outcome
                self._pub.publish(Topics._CMD_FEEDBACK_TOPIC,
                                  CommandFeedback(command="transition",
                                                  args=[command_msg.target, self.name]))
                Logger.localwarn(f"--> Manually triggered outcome {outcome} of concurrency container {self.name}")
                self.on_exit(self.userdata,
                             states=[s for s in self._states if (s.name not in self._returned_outcomes
                                                                 or self._returned_outcomes[s.name] is None)])
                self._returned_outcomes = {}
                self._current_state = None
                self._last_outcome = outcome
                return outcome
            else:
                Logger.localinfo(f"concurrency container {self.name} ")
                self._manual_transition_requested = command_msg

        for state in self._states:
            if state.name in self._returned_outcomes and self._returned_outcomes[state.name] is not None:
                continue  # already done with executing

            if self._manual_transition_requested is not None and self._manual_transition_requested.target == state.name:
                # Transition request applies to this state
                # @TODO - Should we be using path not name here?
                command_msg = self._manual_transition_requested
                self._manual_transition_requested = None  # Reset at this level

                if 0 <= command_msg.outcome < len(state.outcomes):
                    state._force_transition = True
                    outcome = state.outcomes[command_msg.outcome]
                    state._manual_transition_requested = outcome
                    self._returned_outcomes[state.name] = outcome
                    with UserData(reference=self._userdata, remap=self._remappings[state.name],
                                  input_keys=state.input_keys, output_keys=state.output_keys) as userdata:
                        state.on_exit(userdata)

                    # ConcurrencyContainer bypasses normal operatable state handling of manual request, so do that here
                    state._publish_outcome(outcome)

                    self._pub.publish(Topics._CMD_FEEDBACK_TOPIC,
                                      CommandFeedback(command="transition",
                                                      args=[command_msg.target, state.name]))
                    Logger.localerr(f"--> Manually triggered outcome {outcome} ({command_msg.outcome}) "
                                    f"of state {state.name} from inside concurrency {self.name}")
                    continue
                else:
                    Logger.localerr(f"--> Invalid outcome {command_msg.outcome} request for state {state.name} "
                                    f"from inside concurrency {self.name}\n{state.outcomes}")

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

            if state.sleep_duration <= 0:  # ready to execute
                out = self._execute_single_state(state)
                self._returned_outcomes[state.name] = out

            # Track any state that remains as being currently active
            self._current_state.append(state)

            # we want to pass sync requests back up to parent,
            self._inner_sync_request = self._inner_sync_request or state._inner_sync_request

        # Determine concurrency outcome
        outcome = None
        if any(self._returned_outcomes[state.name] == state._preempted_name
               for state in self._states if state.name in self._returned_outcomes):
            return self._preempted_name  # handle preemption if required
        # check conditions
        for item in self._conditions:
            (out, cond) = item
            if all(sn in self._returned_outcomes and self._returned_outcomes[sn] == o for sn, o in cond):
                outcome = out
                break

        if outcome is None:
            return None

        # trigger on_exit for those states that are not done yet
        self.on_exit(self.userdata,
                     states=[s for s in self._states if (s.name not in self._returned_outcomes
                                                         or self._returned_outcomes[s.name] is None)])
        self._returned_outcomes = {}
        # right now, going out of a concurrency container may break sync
        # thus, as a quick fix, explicitly request sync again on any output
        # self._inner_sync_request = True
        self._current_state = None

        if self._is_controlled:
            # reset previously requested outcome if applicable
            if self._last_requested_outcome is not None and outcome is None:
                self._pub.publish(Topics._OUTCOME_REQUEST_TOPIC, OutcomeRequest(outcome=255, target=self.path))
                self._last_requested_outcome = None

            # request outcome because autonomy level is too low
            if (not self._force_transition and self.parent is not None
                and (not self.parent.is_transition_allowed(self.name, outcome)
                     or outcome is not None and self.is_breakpoint)):
                if outcome != self._last_requested_outcome:
                    self._pub.publish(Topics._OUTCOME_REQUEST_TOPIC,
                                      OutcomeRequest(outcome=self.outcomes.index(outcome),
                                                     target=self.path))
                    Logger.localinfo("<-- Want result: %s > %s" % (self.name, outcome))
                    StateLogger.log('flexbe.operator', self, type='request', request=outcome,
                                    autonomy=self.parent.autonomy_level,
                                    required=self.parent.get_required_autonomy(outcome, self))
                    self._last_requested_outcome = outcome
                outcome = None
            elif outcome is not None and outcome in self.outcomes:
                # autonomy level is high enough, report the executed transition
                self._publish_outcome(outcome)
                self._force_transition = False

        self._last_outcome = outcome
        # Logger.localinfo(f"ConcurrencyContainer '{self.name}' returning outcome '{outcome}' (request inner sync)")
        return outcome

    def _execute_single_state(self, state, force_exit=False):
        result = None
        try:
            with UserData(reference=self._userdata, remap=self._remappings[state.name],
                          input_keys=state.input_keys, output_keys=state.output_keys) as userdata:
                state._inner_sync_request = False  # clear any prior sync on call to individual state
                if force_exit:
                    state._entering = True
                    state.on_exit(userdata)
                else:
                    result = state.execute(userdata)
        except Exception as exc:  # pylint: disable=W0703
            result = None
            self._last_exception = exc
            Logger.logerr('ConcurrencyContainer: Failed to execute state %s:\n%s' % (self.current_state_label, str(exc)))
            import traceback  # pylint: disable=C0415
            Logger.localinfo(traceback.format_exc().replace("%", "%%"))

        return result

    def on_enter(self, userdata):  # pylint: disable=W0613
        super().on_enter(userdata)
        for state in self._states:
            # Force on_enter at state level (userdata passed by _execute_single_state)
            state._entering = True  # force state to handle enter on first execute
            state._last_execution = None

    def on_exit(self, userdata, states=None):
        for state in self._states if states is None else states:
            if state in self._returned_outcomes:
                continue  # skip states that already exited themselves
            self._execute_single_state(state, force_exit=True)
        self._current_state = None

    def get_deep_states(self):
        """
        Recursively look for the currently executing states.

        Traverse all state machines down to the terminal child state that is not a container.

        EXCEPT for ConcurrencyContainers.  Those are both active state and container.

        @return: The list of active states (not state machine)
        """
        deep_states = [self]  # Concurrency acts as both state and container for this purpose
        if isinstance(self._current_state, list):
            for state in self._current_state:
                # Internal states (after skipping concurrency container self)
                if isinstance(state, LockableStateMachine):
                    deep_states.extend(state.get_deep_states())
                else:
                    deep_states.append(state)
            # Logger.localinfo(f"Concurrent get_deep_states: {self.name} {[state.path for state in deep_states]}")
            return deep_states
        elif self._current_state is not None:
            Logger.localerr(f"ConcurrentContainer.get_deep_states {self.name} - current state is NOT a list!")
            raise TypeError(f"ConcurrentContainer.get_deep_states {self.name} - current state is NOT a list!")
        # Otherwise, either haven't fully entered, or all have returned outcomes

        return deep_states
