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

"""CalculationState."""
from flexbe_core import EventState, Logger


class CalculationState(EventState):
    """
    Implements a state that can perform a calculation based on userdata.

    calculation is a function which takes exactly one parameter, input_value from userdata,
    and its return value is stored in output_value after leaving the state.

    -- calculation  function    The function that performs the desired calculation.
                                It could be a private function (self.foo) manually defined in a behavior's source code
                                or a Python lambda function (e.g., lambda x: x**2),
                                where x is the argument passed to the function.

    ># input_value  object      Userdata used as input to the calculation function.

    #> output_value object      The result of the calculation.

    <= done                     Indicates completion of the calculation.
    """

    def __init__(self, calculation):
        """Initialize the CalculationState instance."""
        super(CalculationState, self).__init__(outcomes=['done'],
                                               input_keys=['input_value'],
                                               output_keys=['output_value'])
        self._calculation = calculation
        self._calculation_result = None

    def execute(self, userdata):
        """Set userdata with calculation result and return done."""
        userdata.output_value = self._calculation_result
        # nothing to check
        return 'done'

    def on_enter(self, userdata):
        """Call calculation on entering state."""
        if self._calculation is not None:
            try:
                self._calculation_result = self._calculation(userdata.input_value)
            except Exception as e:
                Logger.logwarn('Failed to execute calculation function!\n%s' % str(e))
        else:
            Logger.logwarn('Passed no calculation!')
