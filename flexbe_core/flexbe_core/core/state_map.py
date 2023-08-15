#!/usr/bin/env python

# Copyright 2023 Christopher Newport University
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
#    * Neither the name of the Christopher Newport University nor the names of its
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

import zlib
from flexbe_core.logger import Logger


class StateMap:
    """Standardize hashing of states and provide for storing lookup from id to state instance."""

    __HASH_BITS = 31  # Keep in signed 32-bit range
    # 2**31 - 1 - 256
    __HASH_MASK = 0x7FFFFF00  # 23-bits keeps in signed range, and allows for 256 output encoding

    def __init__(self):
        self._state_map = {}
        self._num_collision_processed = 0

    def __str__(self):
        return (f"State map with {len(self._state_map)} entries"
                "." if self._num_collision_processed == 0 else f" with {self._num_collision_processed} collisions!")

    def __getitem__(self, index):
        """Get existing state if possible, or return None."""
        if index in self._state_map:
            return self._state_map[index]
        else:
            return None

    @classmethod
    def _hash_path(cls, path):
        """
        Convert unique string path to state to numeric ID.

        The hash mask is used to:
        1) Keep as a positive integer value
        2) Mask off the lower 8 bits for use as outcome value for status updates

        Using adler32 for now, but if this causes too many collisions, we may
        consider going to MurmurHash3 (mmh3) library.
        """
        return zlib.adler32(path.encode()) & cls.__HASH_MASK

    def get_path_hash(self, path):
        """Return hash value for path that is in state map."""
        hash_path = path
        hash_val = self._hash_path(hash_path)
        collisions = 0
        while hash_val not in self._state_map:
            collisions += 1
            if collisions > 20:
                return None
            hash_path += path
            hash_val = self._hash_path(hash_path)
        return hash_val

    def add_state(self, path, state):
        """Define state id hash and store in map to state instance."""
        hash_path = path
        if state._state_id is None or state._state_id == -1:
            state._state_id = self._hash_path(hash_path)
            collisions = 0
            while state._state_id in self._state_map:
                collisions += 1
                if collisions > 20:
                    raise KeyError(f"Unable to avoid collisions in StateMap with {path}")
                hash_path += path
                state._state_id = self._hash_path(hash_path)
            self._state_map[state._state_id] = state
            self._num_collision_processed += collisions
        else:
            if state._state_id in self._state_map:
                Logger.error(f"State '{path}' : id={state._state_id} is already in map!")
                raise KeyError(f"Existing state in StateMap with {path}")
            self._state_map[state._state_id] = state

    def get_state(self, state_id):
        """Return reference to state given id."""
        if state_id in self._state_map:
            return self._state_map[state_id]

        Logger.error("State id={state_id} is not in the state map!")
        return None

    @classmethod
    def unhash(cls, hash_code):
        """
        Convert numeric ID to state id and outcome index.

        @return tuple of state id, outcome index
        """
        state_id = hash_code & cls.__HASH_MASK

        # remove outcome offset to return true index
        # https://stackoverflow.com/questions/31151107/how-do-i-do-a-bitwise-not-operation-in-python
        outcome_index = (hash_code & ((1 << cls.__HASH_BITS) - 1 - cls.__HASH_MASK)) - 1

        if outcome_index == -1:
            # No outcome, just a status update
            return state_id, None

        return state_id, outcome_index

    @classmethod
    def hash(cls, state, outcome_index):
        """Convert state id and outcome to hashed identifier for outcome reports."""
        return state._state_id + 1 + outcome_index
