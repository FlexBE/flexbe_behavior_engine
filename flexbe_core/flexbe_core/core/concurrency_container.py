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
from flexbe_core.logger import Logger
from flexbe_core.core.user_data import UserData
from flexbe_core.core.event_state import EventState
from flexbe_core.core.priority_container import PriorityContainer

from flexbe_core.core.operatable_state_machine import OperatableStateMachine


class ConcurrencyContainer(OperatableStateMachine):
    """
    A state machine that can be operated.

    It synchronizes its current state with the mirror and supports some control mechanisms.
    """

    def __init__(self, conditions=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._conditions = conditions if conditions else {}
        self._returned_outcomes = {}

    @property
    def sleep_duration(self):
        """Sleep duration in seconds."""
        sleep_dur = float("inf")
        for state in self._states:
            sleep_dur = min(sleep_dur, state.sleep_duration)

        return sleep_dur

    def _execute_current_state(self):
        # execute all states that are done with sleeping and determine next sleep duration
        self._inner_sync_request = False  # clear prior request for lower level state
        for state in self._states:
            if state.name in self._returned_outcomes and self._returned_outcomes[state.name] is not None:
                continue  # already done with executing

            if (PriorityContainer.active_container is not None
                and not all(a == s for a, s in zip(PriorityContainer.active_container.split('/'),
                                                   state.path.split('/')))):
                if isinstance(state, EventState):
                    state._notify_skipped()
                elif state.get_deep_state() is not None:
                    state.get_deep_state()._notify_skipped()
                continue  # other state has priority

            if state.sleep_duration <= 0:  # ready to execute
                out = self._execute_single_state(state)
                self._returned_outcomes[state.name] = out
                if out:
                    # Track any state with outcome as the current state
                    self._current_state = state

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
        self._inner_sync_request = True
        self._current_state = None
        Logger.localwarn('ConcurrencyContainer %s returning outcome %s (request inner sync)' % (self.name, str(outcome)))
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

    def _enable_ros_control(self):
        state = self._states[0]
        if isinstance(state, EventState):
            state._enable_ros_control()
        if isinstance(state, OperatableStateMachine):
            state._enable_ros_control()

    def _disable_ros_control(self):
        state = self._states[0]
        if isinstance(state, EventState):
            state._disable_ros_control()
        if isinstance(state, OperatableStateMachine):
            state._disable_ros_control()

    def on_enter(self, userdata):  # pylint: disable=W0613
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
