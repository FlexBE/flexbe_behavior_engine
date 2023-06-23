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


from flexbe_core import EventState, Logger


class CheckConditionState(EventState):
    """
    Checks if the given condition is true and returns the corresponding outcome.

    This state can be used if the further control flow of the behavior depends on a simple condition.

    -- predicate    function    The condition whose truth value will be evaluated.
                                Has to expect one parameter which will be set to input_value and return a boolean.

    ># input_value  object      Input to the predicate function.

    <= true                     Returned if the condition evaluates to True
    <= false                    Returned if the condition evaluates to False
    """

    def __init__(self, predicate):
        super(CheckConditionState, self).__init__(outcomes=['true', 'false'],
                                                  input_keys=['input_value'])
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
        try:
            self._outcome = 'true' if self._predicate(userdata.input_value) else 'false'
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(f"Failed to execute condition function!\n  {str(exc)}")
            self._outcome = 'false'
