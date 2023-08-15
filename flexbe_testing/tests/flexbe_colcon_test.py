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


"""Colcon testing for flexbe_testing."""

from os.path import join
import unittest

from flexbe_testing.py_tester import PyTester


class TestFlexBETesting(PyTester):
    """Colcon testing for flexbe_testing."""

    def __init__(self, *args, **kwargs):
        """Construct the unit test instance."""
        super().__init__(*args, **kwargs)

    @classmethod
    def setUpClass(cls):

        PyTester._package = "flexbe_testing"
        PyTester._tests_folder = join("tests", "res")

        super().setUpClass()  # Do this last after setting package and tests folder

    # The tests
    def test_import_only(self):
        """Invoke unittest defined .test file."""
        return self.run_test("import_only")

    def test_add(self):
        """Invoke unittest defined .test file."""
        return self.run_test("test_add")

    # def test_add_bagfile(self):
    #     """ invoke unittest defined .test file """
    #     return self.run_test("test_add_bagfile")

    def test_sub_unavailable(self):
        """Invoke unittest defined .test file."""
        #  This test requires longer than normal wait for valid return value
        #  given 1.5 second timeout in test_sub_state.py
        return self.run_test("sub_unavailable", timeout_sec=2.5, max_cnt=None)

    def test_behavior(self):
        """Invoke unittest defined .test file."""
        return self.run_test("behavior")


if __name__ == '__main__':
    unittest.main()
