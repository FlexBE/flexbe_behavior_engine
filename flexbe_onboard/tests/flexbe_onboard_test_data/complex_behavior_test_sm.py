#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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


###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

__test__ = False  # Do not pytest this class (it is the test!)

from flexbe_INVALID import Autonomy, Behavior, OperatableStateMachine, initialize_flexbe_core

from flexbe_states.calculation_state import CalculationState
from flexbe_states.decision_state import DecisionState
from flexbe_states.log_state import LogState as flexbe_states__LogState
from flexbe_states.wait_state import WaitState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
raise ValueError('TODO: Remove!')
# [/MANUAL_IMPORT]


"""
A more complex behavior for testing the onboard engine.

Created on Mon Mar 30 2020
@author: Philipp Schillinger
"""


class ComplexBehaviorTestSM(Behavior):
    """
    A more complex behavior for testing the onboard engine.

    Note: This behavior contains intentional errors that are fixed by sent modifications.
    """

    __test__ = False  # Do not pytest this class (it is the test!)

    def __init__(self, node):
        """Initialize ComplexBehaviorTestSM instance."""
        super(ComplexBehaviorTestSM, self).__init__()
        self.name = 'Complex Behavior Test'
        self.node = node

        initialize_flexbe_core(node)

        # parameters of this behavior
        self.add_parameter('param', 'value_1')

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create SM instance."""
        # x:67 y:463, x:336 y:160
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['data'])
        _state_machine.userdata.data = 0

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]

        with _state_machine:
            # x:30 y:40
            OperatableStateMachine.add('Wait',
                                       WaitState(wait_time=0.5),
                                       transitions={'done': 'Calculate'},
                                       autonomy={'done': Autonomy.Off})

            # x:36 y:240
            OperatableStateMachine.add('Log Param',
                                       flexbe_states__LogState(text=self.param, severity=2),
                                       transitions={'done': 'Verify Input'},
                                       autonomy={'done': Autonomy.Off})

            # x:32 y:340
            OperatableStateMachine.add('Verify Input',
                                       DecisionState(outcomes=['accepted', 'rejected'],
                                                     conditions=lambda x: 'accepted' if x > 3 else 'rejected'),
                                       transitions={'accepted': 'finished', 'rejected': 'failed'},
                                       autonomy={'accepted': Autonomy.Off, 'rejected': Autonomy.Off},
                                       remapping={'input_value': 'data'})

            # x:28 y:136
            OperatableStateMachine.add('Calculate',
                                       CalculationState(calculation=self._calculate),
                                       transitions={'done': 'Log Param'},
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'input_value': 'data', 'output_value': 'data'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    def _calculate(self, data):
        return data**2
    # [/MANUAL_FUNC]
