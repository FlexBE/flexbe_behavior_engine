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


"""Basic FlexBE State."""
from flexbe_core.core.exceptions import StateError


def _remove_duplicates(input_list):
    output_list = []
    for entry in input_list:
        if entry not in output_list:
            output_list.append(entry)
    return output_list


class State:
    """Basic FlexBE State."""

    def __init__(self, *args, **kwargs):
        self._outcomes = _remove_duplicates(kwargs.get('outcomes', []))
        io_keys = kwargs.get('io_keys', [])
        self._input_keys = _remove_duplicates(kwargs.get('input_keys', []) + io_keys)
        self._output_keys = _remove_duplicates(kwargs.get('output_keys', []) + io_keys)
        # properties of instances of a state machine
        self._name = None
        self._parent = None
        self._inner_sync_request = False  # Any state can generate request, but should be rare

    def __str__(self):
        return self._name

    def execute(self, userdata):
        pass

    @property
    def sleep_duration(self):
        return 0.

    @property
    def outcomes(self):
        return self._outcomes

    @property
    def input_keys(self):
        return self._input_keys

    @property
    def output_keys(self):
        return self._output_keys

    # instance properties

    @property
    def name(self):
        return self._name

    def set_name(self, value):
        if self._name is not None:
            raise StateError("Cannot change the name of a state!")

        self._name = value

    @property
    def parent(self):
        return self._parent

    def set_parent(self, value):
        if self._parent is not None:
            raise StateError("Cannot change the parent of a state!")

        self._parent = value

    @property
    def path(self):
        return "" if self.parent is None else self.parent.path + "/" + self.name
