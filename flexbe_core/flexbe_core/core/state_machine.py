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


"""Implement FlexBE Statemachine."""
from flexbe_core.core.exceptions import StateError, StateMachineError
from flexbe_core.core.state import State
from flexbe_core.core.user_data import UserData
from flexbe_core.logger import Logger


class StateMachine(State):
    """Implement of FlexBE Statemachine."""

    _currently_opened_container = None

    def __init__(self, *args, **kwargs):
        """Initialize the StateMachine instance."""
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
        """Enter the SM."""
        self._previously_opened_container = StateMachine._currently_opened_container
        StateMachine._currently_opened_container = self

    def __exit__(self, *args):
        """Exit the SM."""
        StateMachine._currently_opened_container = self._previously_opened_container
        self._previously_opened_container = None

    def __contains__(self, label):
        """Check if label in this SM."""
        return label in self._labels

    def __getitem__(self, label):
        """Get item corresponding to label."""
        return self._labels[label]

    def __iter__(self):
        """Iterate over states in SM."""
        return iter(state.name for state in self._states)

    # construction

    @staticmethod
    def add(label, state, transitions, remapping=None):
        """Add state to SM."""
        self = StateMachine.get_opened_container()
        if self is None:
            raise StateMachineError("No container opened, activate one first by: 'with state_machine:'")
        if label in self._labels:
            raise StateMachineError("The label '%s' has already been added to this state machine!" % label)
        if label in self._outcomes:
            raise StateMachineError("The label '%s' is an outcome of this state machine!" % label)
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
        """Get open container from this SM."""
        return StateMachine._currently_opened_container

    def wait(self, seconds=None):
        """Wait."""
        # This should not be called; expect to call ros_state_machine version instead!
        Logger.localinfo(f'Error calling StateMachine.wait Dummy wait method for '
                         f"'{self.name}' seconds={seconds}")
        raise RuntimeError(f'Error calling StateMachine.wait Dummy wait method for '
                           f"'{self.name}' seconds={seconds}")

    def spin(self, userdata=None):
        """Spin the SM execute loop."""
        outcome = None
        while True:
            try:
                outcome = self.execute(userdata)
            except Exception as exc:
                Logger.logerr(f"Exception in '{self}' - stopping behavior!")
                Logger.localinfo(f'{exc}')
                self.on_exit(userdata)
                Logger.logerr(f"Exception in '{self}' - {exc}")
                return None

            if outcome is not None:
                break

            self.wait(seconds=self.sleep_duration)

        return outcome

    def execute(self, userdata):
        """Execute the SM."""
        if self._entering:
            self.on_enter(userdata)

        outcome = self._execute_current_state()

        if outcome:
            # Exit this statemachine
            self.on_exit(self._userdata)
            self._publish_outcome(outcome)

        return outcome

    def on_enter(self, userdata):
        """Call on entering the state machine."""
        self.assert_consistent_transitions()
        self._entering = False
        self._exited = False
        self._current_state = self.initial_state
        self._current_state._entering = True  # Force entering action
        self._userdata = userdata if userdata is not None else UserData()
        self._userdata(add_from=self._own_userdata)
        # Logger.localinfo(f"Entering StateMachine '{self.name}' of '{self.path}' "
        #                  f"({self.state_id}) initial state='{self._current_state.name}' ({self.__class__.__name__})")

    def _execute_current_state(self):
        """Execute the currently active state in this SM."""
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
                return target

        return None

    # properties

    @property
    def userdata(self):
        """Return userdata for this SM."""
        return self._own_userdata

    @property
    def current_state(self):
        """Return reference to the currently active state."""
        if self._current_state is not None:
            return self._current_state

        raise StateMachineError('No state active!')

    @property
    def current_state_label(self):
        """Return name of the currently active state."""
        if self._current_state is not None:
            return self._current_state.name

        raise StateMachineError('No state active!')

    @property
    def initial_state(self):
        """Return the initial state."""
        if len(self._states) > 0:
            return self._states[0]

        raise StateMachineError('No states added yet!')

    @property
    def initial_state_label(self):
        """Return the name of the initial state."""
        return self.initial_state.name

    @property
    def sleep_duration(self):
        """Return how long to sleep between execute steps."""
        if self._current_state is not None:
            return self._current_state.sleep_duration
        elif self._entering:
            return -0.1  # No sleep when entering
        return 0.00005  # return some minimal wait

    def get_deep_states(self):
        """
        Recursively look for the currently executing states.

        Traverse all state machines down to the terminal child state that is not a container.
        (Except concurrency containers, which override this method)

        @return: The list of active states (not state machine)
        """
        if isinstance(self._current_state, StateMachine):
            return [self] + self._current_state.get_deep_states()

        # Base case is current_state is not a state machine
        return [self, self._current_state] if self._current_state is not None else [self]  # Return as a list

    # consistency checks

    @property
    def _valid_targets(self):
        """Get list of validate outcomes."""
        return list(self._labels.keys()) + self.outcomes

    def assert_consistent_transitions(self):
        """Validate that transitions are consistent."""
        for transitions in self._transitions.values():
            for transition_target in transitions.values():
                if transition_target not in self._valid_targets:
                    raise StateMachineError("Transition target '%s' missing in %s" %
                                            (transition_target, str(self._valid_targets)))
