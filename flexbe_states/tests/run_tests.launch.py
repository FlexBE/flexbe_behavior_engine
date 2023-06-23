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


"""Flexbe_states testing."""
from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Flexbe_states testing."""
    flexbe_testing_dir = get_package_share_directory('flexbe_testing')
    flexbe_states_test_dir = get_package_share_directory('flexbe_states')

    path = join(flexbe_states_test_dir, "tests")

    testcases = ""
    testcases += join(path, "calculation_state_simple.test") + "\n"
    testcases += join(path, "calculation_state_none.test") + "\n"
    testcases += join(path, "check_condition_state_true.test") + "\n"
    testcases += join(path, "check_condition_state_invalid.test") + "\n"
    testcases += join(path, "decision_state_simple.test") + "\n"
    testcases += join(path, "flexible_calculation_state_simple.test") + "\n"
    testcases += join(path, "input_state_import.test") + "\n"
    testcases += join(path, "log_state_string.test") + "\n"
    testcases += join(path, "log_state_int.test") + "\n"
    # ### ros2 bag issues - testcases += join(path, "log_state_msg.test") + "\n"
    testcases += join(path, "operator_decision_state_suggested.test") + "\n"
    testcases += join(path, "subscriber_state_unavailable.test") + "\n"
    # #### issues with pub/yaml in test testcases += join(path, "subscriber_state_pose.test") + "\n"
    testcases += join(path, "wait_state_short.test") + "\n"

    return LaunchDescription([
        DeclareLaunchArgument("pkg", default_value="flexbe_states"),  # flexbe_testing"),
        DeclareLaunchArgument("testcases", default_value=testcases),
        DeclareLaunchArgument("compact_format", default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(join(flexbe_testing_dir, "launch", "flexbe_testing.launch.py")),
            launch_arguments={
                'package': LaunchConfiguration("pkg"),
                'compact_format': LaunchConfiguration("compact_format"),
                'testcases': LaunchConfiguration("testcases"),
            }.items()
        )
    ])
