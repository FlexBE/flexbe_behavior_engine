#!/usr/bin/env python
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

    def __init__(self, conditions=dict(), *args, **kwargs):
        super(ConcurrencyContainer, self).__init__(*args, **kwargs)
        self._conditions = conditions
        self._returned_outcomes = dict()

    @property
    def sleep_duration(self):
        """
        Sleep duration in seconds
        """
        sleep_dur = float("inf")
        for state in self._states:
            sleep_dur = min(sleep_dur, state.sleep_duration)

        return sleep_dur

    def _execute_current_state(self):
        # execute all states that are done with sleeping and determine next sleep duration
        self._inner_sync_request = False # clear prior
        for state in self._states:
            if state.name in list(self._returned_outcomes.keys()) and self._returned_outcomes[state.name] is not None:
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
                oc = self._execute_single_state(state)
                self._returned_outcomes[state.name] = oc
                if oc:
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
            (oc, cond) = item
            if all(sn in self._returned_outcomes and self._returned_outcomes[sn] == o for sn, o in cond):
                outcome = oc
                break

        if outcome is None:
            return None

        # trigger on_exit for those states that are not done yet
        self.on_exit(self.userdata,
                     states=[s for s in self._states if (s.name not in list(self._returned_outcomes.keys()) or
                                                         self._returned_outcomes[s.name] is None)])
        self._returned_outcomes = dict()
        # right now, going out of a concurrency container may break sync
        # thus, as a quick fix, explicitly request sync again on any output
        self._inner_sync_request = True
        self._current_state = None
        Logger.warning('ConcurrencyContainer %s returning outcome %s (request inner sync)' % (self.name, str(outcome)))
        return outcome

    def _execute_single_state(self, state, force_exit=False):
        result = None
        try:
            with UserData(reference=self._userdata, remap=self._remappings[state.name],
                          input_keys=state.input_keys, output_keys=state.output_keys) as userdata:
                state._inner_sync_request = False  # clear any prior sync on call
                if force_exit:
                    state._entering = True
                    state.on_exit(userdata)
                else:
                    result = state.execute(userdata)
        except Exception as exc:
            result = None
            self._last_exception = exc
            Logger.logerr('ConcurrencyContainer: Failed to execute state %s:\n%s' % (self.current_state_label, str(exc)))
            import traceback
            Logger.localinfo(traceback.format_exc())
            
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

    def on_enter(self, userdata):
        for state in self._states :
            #if not state._entering:
            #    Logger.logerr('Reset entering flag for child state: %s' % (state.name))
            state._entering = True  # force state to handle enter on first execute
            state._last_execution = None

    def on_exit(self, userdata, states=None):
        for state in self._states if states is None else states:
            if state in self._returned_outcomes:
                continue  # skip states that already exited themselves
            self._execute_single_state(state, force_exit=True)
