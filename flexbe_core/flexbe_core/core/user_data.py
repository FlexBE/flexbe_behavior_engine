#!/user/bin/env python

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


"""UserData Class."""
from copy import deepcopy
from flexbe_core.core.exceptions import UserDataError


class UserData:
    """UserData Class."""

    def __init__(self, reference=None, input_keys=None, output_keys=None, remap=None):
        self._data = {}
        self._reference = reference if reference is not None else {}
        self._input_keys = input_keys
        self._output_keys = output_keys
        self._remap = remap or {}
        self._hashes = {}

    def __enter__(self):
        return self

    def __exit__(self, *args):
        for key, value in self._hashes.items():
            if value != hash(repr(self._data[key])):
                raise UserDataError("Illegally modified input-only key '%s', declare it as output." % key)

    def __contains__(self, key):
        if key in self._data:
            return True
        elif self._input_keys is not None and key not in self._input_keys:
            return False

        return self._remap.get(key, key) in self._reference

    def __getitem__(self, key):
        if key in self._data:
            return self._data[key]
        if key not in self:
            raise UserDataError("Key '%s' cannot be accessed, declare it as input key for read access." % key
                                if self._input_keys is not None and key not in self._input_keys else
                                "No data found for key '%s'" % key)
        value = self._reference[self._remap.get(key, key)]
        if self._output_keys is not None and key not in self._output_keys:
            self._data[key] = value
            self._hashes[key] = hash(repr(value))
            if getattr(value.__class__, "_has_header", False):
                # This is specific to rospy: If the value here is a message and has a header,
                #   it will automatically be modified during publishing by rospy.
                #   So to avoid hash issues, we need to return a copy.
                value = deepcopy(value)
        return value

    def __setitem__(self, key, value):
        if self._output_keys is not None and key in self._output_keys:
            self._reference[self._remap.get(key, key)] = value
        self._data[key] = value

    def __getattr__(self, key):
        """
        For use by instance dot operator.

        Presume user data unless prefixed with '_'.
        then treat as regular (private) instance attribute
        """
        if key.startswith('_'):
            # Treat as a regular private instance attribute
            return super().__getattribute__(key)

        # Treat
        return self[key]

    def __setattr__(self, key, value):
        """
        For use by instance dot operator.

        Presume user data unless prefixed with '_'
        then treat as regular (private) instance attribute
        """
        if key.startswith('_'):
            # Treat as a regular private instance attribute
            super().__setattr__(key, value)
            return

        if self._output_keys is not None and key not in self._output_keys:
            raise UserDataError("Key '%s' cannot be set, declare it as output key for write access." % key)
        self[key] = value

    def __call__(self, reference=None, add_from=None, update_from=None, remove_key=None):
        self._reference = reference if reference is not None else self._reference
        if isinstance(add_from, UserData):
            for key, value in add_from._data.items():
                if key not in self._data:
                    self._data[key] = value
        if isinstance(update_from, UserData):
            for key, value in update_from._data.items():
                self._data[key] = value
        if remove_key is not None and remove_key in self._data:
            del self._data[remove_key]

    def __len__(self):
        return len(self._data) + len(self._reference)

    def __str__(self):
        if isinstance(self._reference, UserData):
            data_str = '\n  '.join(str(self._reference).split('\n'))
        else:
            data_str = str(self._reference)
        return ("UserData object with %d data entries:\n"
                "  Input Keys: %s\n  Output Keys: %s\n  Data: %s\n  Remapping: %s\n  Reference: %s"
                % (len(self), str(self._input_keys), str(self._output_keys), str(self._data),
                   str(self._remap), data_str))
