# Copyright 2024 Christopher Newport University
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
#    * Neither the name of the Christopher Newport University nor the names of its
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

from flexbe_core import Logger
from flexbe_core.core import State, StateMachine
from flexbe_core.core import StateMap

from flexbe_mirror.mirror_state import MirrorState
from flexbe_mirror.mirror_state_machine import MirrorStateMachine


class MirrorConcurrencyContainer(MirrorStateMachine):
    """Manage updates of ConcurrencyContainer in the FlexBE mirror in response to changes."""

    def __init__(self, target_name, target_path, *args, **kwargs):
        """Initialize MirrorConcurrrencyContainer instance."""
        MirrorStateMachine.__init__(self, target_name, target_path, *args, **kwargs)
        self._returned_outcomes = {}

    def on_enter_mirror(self, userdata):
        """Enter mirror concurrency container state."""
        self.assert_consistent_transitions()
        self._current_state = self._states[:]
        self._userdata = None  # Mirror does not use user data
        self._last_outcome = None
        self._entering = False
        for state in self._states:
            # Force on_enter at state level
            state._entering = True  # force state to handle enter on first execute
            state._last_execution = None

        MirrorState.publish_update(self.state_id)

    def on_exit_mirror(self, userdata, desired_outcome=-1, states=None):
        """Exit state and prepare for next entry (outcome -1 means preempt)."""
        self._entering = True
        for state in self._states if states is None else states:
            if state in self._returned_outcomes:
                continue  # skip states that already exited themselves
            state._entering = True
            state.on_exit_mirror(userdata, -1)  # preempted

        self._current_state = None
        self._returned_outcomes = {}
        if desired_outcome != -1:
            if desired_outcome == StateMap._MAX_OUTCOME:
                self._last_outcome = State._preempted_name
            else:
                self._last_outcome = self.outcomes[desired_outcome]
        MirrorState.publish_update(self.state_id + 255)  # publish that we "entered" container to exit
        return self._last_outcome

    def execute_mirror(self, userdata):
        """
        Define custom mirror execute method.

        Does not need to handle user state like onboard state machine
        """
        if self._entering:
            self.on_enter_mirror(userdata)

        if MirrorState._last_state_id == self.state_id:
            # Handle outcome of this internal SM
            if self._last_outcome is not None:
                Logger.localwarn(f"Mirror SM concurrency execute for '{self.name.replace('_mirror', '')}' of "
                                 f" '{self.path.replace('_mirror', '')}' ({self.state_id}) : "
                                 f"Already processed outcome='{self._last_outcome}' for "
                                 f'outcome index={MirrorState._last_state_outcome} - reprocessing anyway')

            MirrorState._last_state_id = None  # Flag that the message was handled
            if MirrorState._last_state_outcome is not None:
                desired_outcome = MirrorState._last_state_outcome
                MirrorState._last_state_outcome = None  # Flag that the message was handled
                return self.on_exit_mirror(userdata, desired_outcome,
                                           states=[s for s in self._states if (s.name not in self._returned_outcomes
                                                                               or self._returned_outcomes[s.name] is None)
                                                   ])

        return self._execute_current_state_mirror(userdata)

    def _execute_current_state_mirror(self, userdata):
        self._current_state = []  # Concurrent updates active states list each cycle
        # Handle interior containers
        for state in self._states:
            if state.name in self._returned_outcomes and self._returned_outcomes[state.name] is not None:
                continue  # already done with executing

            out = state.execute_mirror(userdata)
            if out is not None:
                self._returned_outcomes[state.name] = out
                MirrorStateMachine._execute_flag = True  # Spin it again on change
            else:
                self._current_state.append(state)
        # Handle container return in execute
        if len(self._current_state) == 0:
            # No unexited states in concurrent, so notify we have returned to concurrent level
            # Logger.localinfo(f"Inside CC '{self}' - no active internal - publish update '{self._target_path}'")
            MirrorState.publish_update(self.state_id)  # Notify back at top-level before exit
        return None

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
                if isinstance(state, StateMachine):
                    deep_states.extend(state.get_deep_states())
                else:
                    deep_states.append(state)
        elif self._current_state is not None:
            Logger.localerr(f"MirrorConcurrentContainer.get_deep_states '{self.name}' ({self._current_state.state_id})\n"
                            f" - current state is NOT a list! Error type='{type(self._current_state)}'")
            Logger.localerr(f"    '{self._current_state.name}' ({self._current_state.state_id})")
            raise TypeError(f"MirrorConcurrentContainer.get_deep_states '{self.name}' - "
                            f"current state is NOT a list! Error type='{type(self._current_state)}'")
        return deep_states
