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


"""Pytest testing for flexbe_states."""


from flexbe_testing.py_tester import PyTester


class TestFlexBEStates(PyTester):
    """Pytest testing for flexbe_states."""

    def __init__(self, *args, **kwargs):
        """Initialize unit test."""
        super().__init__(*args, **kwargs)

    @classmethod
    def setUpClass(cls):

        PyTester._package = "flexbe_states"
        PyTester._tests_folder = "tests"

        super().setUpClass()  # Do this last after setting package and tests folder

    # The tests
    def test_calculation_state_simple(self):
        """Run FlexBE unit test given .test file."""
        self.run_test("calculation_state_simple")

    def test_calculation_state_none(self):
        """Run FlexBE unit test given .test file."""
        self.run_test("calculation_state_none")

    def test_check_condition_state_true(self):
        """Run FlexBE unit test given .test file."""
        self.run_test("check_condition_state_true")

    def test_check_condition_state_invalid(self):
        """Run FlexBE unit test given .test file."""
        self.run_test("check_condition_state_invalid")

    def test_decision_state_simple(self):
        """Run FlexBE unit test given .test file."""
        self.run_test("decision_state_simple")

    def test_flexible_calculation_state_simple(self):
        """Run FlexBE unit test given .test file."""
        self.run_test("flexible_calculation_state_simple")

    def test_input_state_import(self):
        """Run FlexBE unit test given .test file."""
        self.run_test("input_state_import")

    def test_log_state_string(self):
        """Run FlexBE unit test given .test file."""
        self.run_test("log_state_string")

    def test_log_state_int(self):
        """Run FlexBE unit test given .test file."""
        self.run_test("log_state_int")

    def test_operator_decision_state_suggested(self):
        """Run FlexBE unit test given .test file."""
        self.run_test("operator_decision_state_suggested")

    def test_subscriber_state_unavailable(self):
        """Run FlexBE unit test given .test file."""
        self.run_test("subscriber_state_unavailable")

    def test_wait_state_short(self):
        """
        Run FlexBE unit test given .test file.

        This test requires longer wait than normal
        """
        self.run_test("wait_state_short", timeout_sec=1.5, max_cnt=5000)

    # Tests with issues
    # # #### issues with pub/yaml loading Pose - subscriber_state_pose
    # # ### ros2 bag issues  - log_state_msg
