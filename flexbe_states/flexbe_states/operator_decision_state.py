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

"""OperatorDecisionState."""
from flexbe_core import EventState, Logger


class OperatorDecisionState(EventState):
    """
    Implements a state where the operator has to manually choose an outcome.

    Autonomy Level of all outcomes should be set to Full,
    because this state is not able to choose an outcome on its own.
    Only exception is the suggested outcome, which will be returned immediately by default.
    This state can be used to create alternative execution paths
    by setting the suggestion to High autonomy instead of Full.

    -- outcomes     string[]    A list of all possible outcomes of this state.
    -- hint         string      Text displayed to the operator to give instructions how to decide.
    -- suggestion   string      The outcome which is suggested.
                                Will be returned if the level of autonomy is high enough.
    """

    def __init__(self, outcomes, hint=None, suggestion=None):
        """Initialize OperatorDecisionState instance."""
        super(OperatorDecisionState, self).__init__(outcomes=outcomes)
        self._hint = hint
        self._suggestion = suggestion

    def execute(self, userdata):
        """Execute until operator defines outcome."""
        if self._suggestion is not None and self._suggestion in self._outcomes:
            return self._suggestion

        return None

    def on_enter(self, userdata):
        """Call on entering state, and published suggested hint."""
        if self._hint is not None:
            Logger.loghint(self._hint)
