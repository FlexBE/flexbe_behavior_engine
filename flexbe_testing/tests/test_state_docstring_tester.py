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
#    * Neither the name of Christopher Newport University nor the names of its
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

"""Tests for FlexBE state docstring validation helpers."""

from flexbe_testing.state_docstring_tester import main
from flexbe_testing.state_docstring_tester import validate_state_docstrings


def _write_state(tmp_path, type_name):
    state_path = tmp_path / 'choice_state.py'
    state_path.write_text(
        'class ChoiceState(EventState):\n'
        '    """Exercise type parsing.\n'
        f'    -- value {type_name} Value to use.\n'
        '    <= done Finished.\n'
        '    """\n'
        '    def __init__(self, value):\n'
        "        super().__init__(outcomes=['done'])\n",
        encoding='utf-8',
    )
    return state_path


def test_state_docstring_type_allows_pipe_without_whitespace(tmp_path):
    """Accept union-style type names when pipe separators are part of the token."""
    assert validate_state_docstrings([_write_state(tmp_path, 'str|bool')]) == []


def test_state_docstring_type_rejects_pipe_with_whitespace(tmp_path):
    """Reject union-style type names with whitespace around pipe separators."""
    errors = validate_state_docstrings([_write_state(tmp_path, 'str | bool')])

    assert len(errors) == 1
    assert 'without whitespace' in errors[0]


def test_state_docstring_tester_command_succeeds_for_explicit_path(tmp_path, capsys):
    """Return success from the command entry point for valid explicit paths."""
    state_path = _write_state(tmp_path, 'str|bool')

    assert main(['example_pkg', '--quiet', '--path', str(state_path)]) == 0
    assert capsys.readouterr().err == ''


def test_state_docstring_tester_command_fails_for_explicit_path(tmp_path, capsys):
    """Return failure from the command entry point for invalid explicit paths."""
    state_path = _write_state(tmp_path, 'str | bool')

    assert main(['example_pkg', '--quiet', '--path', str(state_path)]) == 1
    assert 'without whitespace' in capsys.readouterr().err
