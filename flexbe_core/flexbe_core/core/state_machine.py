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


"""Implement FlexBE Statemachine."""
from flexbe_core.logger import Logger
from flexbe_core.core.state import State
from flexbe_core.core.user_data import UserData
from flexbe_core.core.exceptions import StateError, StateMachineError


class StateMachine(State):
    """Implement of FlexBE Statemachine."""

    _currently_opened_container = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._states = []
        self._labels = {}
        self._transitions = {}
        self._remappings = {}
        self._current_state = None
        self._own_userdata = UserData()
        self._userdata = None
        self._previously_opened_container = None

    def __enter__(self):
        self._previously_opened_container = StateMachine._currently_opened_container
        StateMachine._currently_opened_container = self

    def __exit__(self, *args):
        StateMachine._currently_opened_container = self._previously_opened_container
        self._previously_opened_container = None

    def __contains__(self, label):
        return label in self._labels

    def __getitem__(self, label):
        return self._labels[label]

    def __iter__(self):
        return iter(state.name for state in self._states)

    # construction

    @staticmethod
    def add(label, state, transitions, remapping=None):
        self = StateMachine.get_opened_container()
        if self is None:
            raise StateMachineError("No container opened, activate one first by: 'with state_machine:'")
        if label in self._labels:
            raise StateMachineError("The label %s has already been added to this state machine!" % label)
        if label in self._outcomes:
            raise StateMachineError("The label %s is an outcome of this state machine!" % label)
        # add state to this state machine
        self._states.append(state)
        self._labels[label] = state
        self._transitions[label] = transitions
        self._remappings[label] = remapping or {}
        # update state instance
        state.set_name(label)
        state.set_parent(self)

    @staticmethod
    def get_opened_container():
        return StateMachine._currently_opened_container

    def wait(self, seconds=None):
        # This should not be called; expect to call ros_state_machine version instead!
        Logger.localinfo(f"Error calling StateMachine.wait Dummy wait method for "
                         f"{self.name} seconds={seconds}")
        raise RuntimeError(f"Error calling StateMachine.wait Dummy wait method for "
                           f"{self.name} seconds={seconds}")

    def spin(self, userdata=None):
        outcome = None
        while True:
            outcome = self.execute(userdata)

            if outcome is not None:
                break

            self.wait(seconds=self.sleep_duration)

        return outcome

    def execute(self, userdata):
        if self._current_state is None:
            self.assert_consistent_transitions()
            self._current_state = self.initial_state
            self._userdata = userdata if userdata is not None else UserData()
            self._userdata(add_from=self._own_userdata)
            # Logger.localinfo(f"Entering StateMachine {self.name} ({self._state_id}) initial state='{self._current_state.name}'")
        outcome = self._execute_current_state()
        return outcome

    def _execute_current_state(self):
        with UserData(reference=self._userdata, remap=self._remappings[self._current_state.name],
                      input_keys=self._current_state.input_keys, output_keys=self._current_state.output_keys
                      ) as userdata:
            self._current_state._inner_sync_request = False  # clear any prior downstream sync request
            outcome = self._current_state.execute(userdata)

        # Pass any sync request to parent
        self._inner_sync_request = self._inner_sync_request or self._current_state._inner_sync_request

        if outcome is not None:
            try:
                target = self._transitions[self._current_state.name][outcome]
            except KeyError as exc:
                err_msg = f"Returned outcome '{outcome}' is not registered as a transition from '{self._current_state}'"
                raise StateError(err_msg) from exc

            self._current_state = self._labels.get(target)
            if self._current_state is None:
                Logger.localinfo(f" SM '{self.name}' ({self.id}) returning '{target}' ")
                return target
            # else:
            #     Logger.localinfo(f" SM '{self.name}' ({self.id}) updated current state to "
            #                      f"'{self._current_state.name}' ({self._current_state._state_id}) given outcome='{target}' ")

        return None

    # properties

    @property
    def userdata(self):
        return self._own_userdata

    @property
    def current_state(self):
        if self._current_state is not None:
            return self._current_state

        raise StateMachineError("No state active!")

    @property
    def current_state_label(self):
        if self._current_state is not None:
            return self._current_state.name

        raise StateMachineError("No state active!")

    @property
    def initial_state(self):
        if len(self._states) > 0:
            return self._states[0]

        raise StateMachineError("No states added yet!")

    @property
    def initial_state_label(self):
        return self.initial_state.name

    @property
    def sleep_duration(self):
        if self._current_state is not None:
            return self._current_state.sleep_duration

        return 0.00005  # return some minimal wait

    def get_deep_states(self):
        """
        Recursively look for the currently executing states.

        Traverse all state machines down to the terminal child state that is not a container.
        (Except concurrency containers, which override this method)

        @return: The list of active states (not state machine)
        """
        if isinstance(self._current_state, StateMachine):
            return self._current_state.get_deep_states()

        # Base case is current_state is not a state machine
        return [self._current_state] if self._current_state is not None else []  # Return as a list

    # consistency checks

    @property
    def _valid_targets(self):
        return list(self._labels.keys()) + self.outcomes

    def assert_consistent_transitions(self):
        for transitions in self._transitions.values():
            for transition_target in transitions.values():
                if transition_target not in self._valid_targets:
                    raise StateMachineError("Transition target '%s' missing in %s" %
                                            (transition_target, str(self._valid_targets)))
