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


"""DecisionState."""
from flexbe_core import EventState, Logger


class DecisionState(EventState):
    """
    Evaluates a condition function in order to return one of the specified outcomes.

    This state can be used if the further control flow of the behavior depends on an advanced condition.

    -- outcomes     string[]    A list containing all possible outcomes of this state
    -- conditions   function    Implements the condition check and returns one of the available outcomes.
                                Has to expect one parameter which will be set to input_value.

    ># input_value  object      Input to the condition function.
    """

    def __init__(self, outcomes, conditions):
        """Construct instance."""
        super(DecisionState, self).__init__(outcomes=outcomes,
                                            input_keys=['input_value'])
        self._conditions = conditions

    def execute(self, userdata):
        """Execute state and check conditions and return associated outcome."""
        if self._conditions is not None:
            outcome = None
            try:
                outcome = str(self._conditions(userdata.input_value))
            except Exception as e:
                Logger.logwarn('Passed no function as predicate!\n%s' % str(e))
                outcome = None
            if outcome is not None and outcome in self._outcomes:
                return outcome
        return None
