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


"""FlexibleCheckConditionState."""
from flexbe_core import EventState, Logger


class FlexibleCheckConditionState(EventState):
    """
    Implements a state that checks if the given condition is true.

    Uses multiple userdata inputs provided as a list to the calculation function and
    returns the corresponding outcome.

    This state can be used if the further control flow of the behavior depends on a simple condition.

    -- predicate    function    The condition whose truth value will be evaluated.
                                It could be a private function (self.foo) manually defined in a behavior's source code
                                or a lambda function (e.g., lambda x: x[0]^2 + x[1]^2).
    -- input_keys   string[]    List of available input keys.

    ># input_keys   object[]    Input(s) to the calculation function as a list of userdata.
                                The individual inputs can be accessed as list elements (see lambda expression example).

    <= true                     Returned if the condition evaluates to True
    <= false                    Returned if the condition evaluates to False
    """

    def __init__(self, predicate, input_keys):
        """Construct instance."""
        super(FlexibleCheckConditionState, self).__init__(outcomes=['true', 'false'],
                                                          input_keys=input_keys)
        self._predicate = None
        if callable(predicate):
            self._predicate = predicate
        elif isinstance(predicate, str):
            if "__" in predicate:
                Logger.logwarn(f"potentially unsafe code in predicate '{predicate}' - use caution if executing!")
            try:
                self._predicate = eval(predicate)  # Assumes behavior is from a trusted source!
            except Exception as exc:  # pylint: disable=W0703
                Logger.logwarn(f"Failed to convert predicate to callable function!\n  {str(exc)}")

        self._outcome = 'false'

    def execute(self, userdata):
        return self._outcome

    def on_enter(self, userdata):
        if self._predicate is not None:
            try:
                self._outcome = "true" if self._predicate([userdata[key] for key in self._input_keys]) else 'false'
            except Exception as exc:  # pylint: disable=W0703
                Logger.logwarn(f"failed to execute condition function!\n  {str(exc)}")
        else:
            Logger.logwarn('Passed no predicate!')
