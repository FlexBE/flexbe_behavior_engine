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


"""
Launch file used for testing of FlexBE states and behaviors.

Generally invoked from test launch, not launched directly here.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "testcases",
            default_value=""),
        DeclareLaunchArgument(
            "package",
            default_value=""),
        DeclareLaunchArgument(
            "print_debug_positive",
            default_value="true"),
        DeclareLaunchArgument(
            "print_debug_negative",
            default_value="true"),
        DeclareLaunchArgument(
            "mute_info",
            default_value="false"),
        DeclareLaunchArgument(
            "mute_warn",
            default_value="false"),
        DeclareLaunchArgument(
            "mute_error",
            default_value="false"),
        DeclareLaunchArgument(
            "compact_format",
            default_value="false"),
        Node(
            package="flexbe_testing", executable="testing_node", output="screen",
            name="flexbe_testing",
            arguments=[LaunchConfiguration("testcases")],
            parameters=[{"~package": LaunchConfiguration("package"),
                         "~print_debug_positive": LaunchConfiguration("print_debug_positive"),
                         "~print_debug_negative": LaunchConfiguration("print_debug_negative"),
                         "~mute_info": LaunchConfiguration("mute_info"),
                         "~mute_warn": LaunchConfiguration("mute_warn"),
                         "~mute_error": LaunchConfiguration("mute_error"),
                         "~compact_format": LaunchConfiguration("compact_format")}])
    ])
