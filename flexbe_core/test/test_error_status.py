#!/usr/bin/env python3

# Copyright 2026  Christopher Newport University
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


"""Unit tests for exception to BEStatus mapping."""

import unittest

from flexbe_core.core import BehaviorLoadError, ProxyTypeError, SyncError, TransitionError
from flexbe_core.core import StateError
from flexbe_core.core.error_status import map_exception_to_bestatus

from flexbe_msgs.msg import BEStatus


class TestErrorStatus(unittest.TestCase):
    """Validate mapping behavior for typed exceptions."""

    def test_proxy_error_maps_to_error(self):
        """Verify proxy type errors map to ERROR."""
        self.assertEqual(map_exception_to_bestatus(ProxyTypeError('bad type')), BEStatus.ERROR)

    def test_behavior_load_error_maps_to_error(self):
        """Verify behavior load errors map to ERROR."""
        self.assertEqual(map_exception_to_bestatus(BehaviorLoadError('load failed')), BEStatus.ERROR)

    def test_sync_error_maps_to_error(self):
        """Verify sync errors map to ERROR."""
        self.assertEqual(map_exception_to_bestatus(SyncError('sync failed')), BEStatus.ERROR)

    def test_transition_error_maps_to_error(self):
        """Verify transition errors map to ERROR."""
        self.assertEqual(map_exception_to_bestatus(TransitionError('transition failed')), BEStatus.ERROR)

    def test_state_error_maps_to_failed(self):
        """Verify state errors map to FAILED."""
        self.assertEqual(map_exception_to_bestatus(StateError('state failed')), BEStatus.FAILED)

    def test_unknown_exception_uses_default(self):
        """Verify unknown exceptions fall back to provided default."""
        self.assertEqual(map_exception_to_bestatus(RuntimeError('boom'), default=BEStatus.ERROR), BEStatus.ERROR)


if __name__ == '__main__':
    unittest.main()
