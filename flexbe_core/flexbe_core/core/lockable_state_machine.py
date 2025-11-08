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


"""Implement of LockableStateMachine that can prevent transition."""

from flexbe_core.core.ros_state_machine import RosStateMachine


class LockableStateMachine(RosStateMachine):
    """
    A state machine that can be locked.

    When locked, no transition can be done regardless of the resulting outcome.
    However, if any outcome would be triggered, the outcome will be stored
    and the state won't be executed anymore until it is unlocked and the stored outcome is set.
    """

    path_for_switch = None

    def __init__(self, *args, **kwargs):
        """Initialize LockableStateMachine instance."""
        super().__init__(*args, **kwargs)
        self._locked = False

    def _is_internal_transition(self, transition_target):
        """Is a valid transition."""
        return transition_target in self._labels

    def transition_allowed(self, state, outcome):
        """Is transition allowed."""
        if outcome is None or outcome == 'None':
            return True
        transition_target = self._transitions[state].get(outcome)
        return (self._is_internal_transition(transition_target)
                or (not self._locked
                    and (self.parent is None
                         or self.parent.transition_allowed(self.name, transition_target))))

    # for switching

    def execute(self, userdata):
        """Execute lockable SM logic."""
        if (LockableStateMachine.path_for_switch is not None
                and LockableStateMachine.path_for_switch.startswith(self.path)):
            path_segments = LockableStateMachine.path_for_switch.replace(self.path, '', 1).split('/')
            wanted_state = path_segments[1]
            self._current_state = self._labels[wanted_state]
            if len(path_segments) <= 2:
                LockableStateMachine.path_for_switch = None
        return super().execute(userdata)

    def replace_userdata(self, userdata):
        """Replace userdata."""
        self._userdata = userdata

    def replace_state(self, state):
        """Replace state (while locked)."""
        old_state = self._labels[state.name]
        state._parent = old_state._parent
        self._states[self._states.index(old_state)] = state
        self._labels[state.name] = state

    def remove_state(self, state):
        """Remove state."""
        del self._labels[state.name]
        self._states.remove(state)

    # for locking

    def lock(self, st_id):
        """Lock this state."""
        if st_id == self.state_id:
            self._locked = True
            return True

        if self._parent is not None:
            return self._parent.lock(st_id)

        return False

    def unlock(self, st_id):
        """Unlock this state."""
        if st_id == self.state_id:
            self._locked = False
            return True

        if self._parent is not None:
            return self._parent.unlock(st_id)

        return False

    def is_locked(self):
        """Is this state locked."""
        return self._locked

    def is_locked_inside(self):
        """Is interior state locked."""
        if self._locked:
            return True
        for state in self._states:
            result = False
            if isinstance(state, LockableStateMachine):
                result = state.is_locked_inside()
            else:
                result = state.is_locked()
            if result is True:
                return True
        return False

    def get_locked_state(self):
        """Get state that is locked inside."""
        if self._locked:
            return self
        for state in self._states:
            if state.is_locked():
                return state
            if isinstance(state, LockableStateMachine):
                return state.get_locked_state()
        return None
