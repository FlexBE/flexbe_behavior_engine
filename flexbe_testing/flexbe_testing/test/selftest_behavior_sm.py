#!/usr/bin/env python
# -*- coding: utf-8 -*-

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


"""Content is only intended for the flexbe_testing self-test."""

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, EventState, Logger


class SelftestBehaviorSM(Behavior):
    """Simple behavior for the flexbe_testing self-test of behaviors."""

    def __init__(self, node):
        super(SelftestBehaviorSM, self).__init__()
        self.name = 'Selftest Behavior'

        self.node = node
        OperatableStateMachine.initialize_ros(self.node)
        SelftestBehaviorSM._CalculationState.initialize_ros(self.node)
        SelftestBehaviorSM._DecisionState.initialize_ros(self.node)

        # parameters of this behavior
        self.value = None  # avoid pylint error
        self.add_parameter('value', 'wrong')

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'],
                                                input_keys=['data'],
                                                output_keys=['result'])
        _state_machine.userdata.data = None
        _state_machine.userdata.result = None

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        def calc_fcn(x):
            return x * 2

        # flake8 says use def not lambda
        # Compares the loaded parameter value (not default 'wrong' above)
        # condition_fcn = lambda x: 'finished' if self.value == 'correct' else 'failed'
        def condition_fcn(*args):
            Logger.loginfo(f'Evaluate condition with input {type(args)} {args}')
            # Note: We are defining function based on current SM self not the args called later!
            return 'finished' if self.value == 'correct' else 'failed'

        # [/MANUAL_CREATE]

        with _state_machine:
            OperatableStateMachine.add('Modify Data',
                                       SelftestBehaviorSM._CalculationState(calculation=calc_fcn),
                                       transitions={'done': 'Decide Param'},
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'input_value': 'data', 'output_value': 'result'})

            OperatableStateMachine.add('Decide Param',
                                       SelftestBehaviorSM._DecisionState(outcomes=['finished', 'failed'],
                                                                         conditions=condition_fcn),
                                       transitions={'finished': 'finished', 'failed': 'failed'},
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'input_value': 'data'})
        return _state_machine

    class _CalculationState(EventState):
        """Copy of the flexbe_states.CalculationState for use in the test behavior."""

        def __init__(self, calculation):
            super(SelftestBehaviorSM._CalculationState, self).__init__(outcomes=['done'],
                                                                       input_keys=['input_value'],
                                                                       output_keys=['output_value'])
            self._calculation = calculation
            self._calculation_result = None

        def execute(self, userdata):
            userdata.output_value = self._calculation_result
            return 'done'

        def on_enter(self, userdata):
            if self._calculation is not None:
                try:
                    self._calculation_result = self._calculation(userdata.input_value)
                except Exception as exc:  # pylint: disable=W0703
                    Logger.logwarn('Failed to execute calculation function!\n%s' % str(exc))
            else:
                Logger.logwarn('Passed no calculation!')

    class _DecisionState(EventState):
        """Copy of the flexbe_states.DecisionState for use in the test behavior."""

        def __init__(self, outcomes, conditions):
            super(SelftestBehaviorSM._DecisionState, self).__init__(outcomes=outcomes, input_keys=['input_value'])
            self._conditions = conditions

        def execute(self, userdata):
            if self._conditions is not None:
                outcome = None
                try:
                    Logger.logwarn(f'_DecisionState execute with UD = {userdata.input_value}')
                    outcome = str(self._conditions(userdata.input_value))
                    Logger.logwarn(f'_DecisionState execute: outcome = {outcome}')
                except Exception as exc:  # pylint: disable=W0703
                    Logger.logwarn('Passed no function as predicate!\n%s' % str(exc))
                    Logger.logwarn(f' conditions {type(self._conditions)}')
                    Logger.logwarn(f'        {self._conditions}')
                    outcome = None
                if outcome is not None and outcome in self._outcomes:
                    return outcome

            return None

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
