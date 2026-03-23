#!/usr/bin/env python3

# Copyright 2026 Christopher Newport University
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

"""Unit tests for PriorityContainer lifecycle delegation."""

import unittest
from unittest.mock import patch

from flexbe_core.core.priority_container import PriorityContainer


class TestPriorityContainerLifecycle(unittest.TestCase):
    """Validate lifecycle delegation behavior for priority container."""

    def test_on_exit_calls_parent_on_exit(self):
        """PriorityContainer.on_exit should delegate to parent on_exit, not on_enter."""
        container = object.__new__(PriorityContainer)
        container._name = 'priority'

        with patch('flexbe_core.core.priority_container.OperatableStateMachine.on_enter') as on_enter, \
                patch('flexbe_core.core.priority_container.OperatableStateMachine.on_exit') as on_exit, \
                patch('flexbe_core.core.priority_container.Logger.localinfo'):
            PriorityContainer.on_exit(container, None)

        on_exit.assert_called_once_with(None)
        on_enter.assert_not_called()


if __name__ == '__main__':
    unittest.main()
