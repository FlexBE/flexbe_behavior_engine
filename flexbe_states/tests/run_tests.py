import os
import sys
import ament_index_python
import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import launch_testing_ros
import pytest

@pytest.mark.rostest
def generate_test_description():
    path_to_test = os.path.dirname(__file__)
    flexbe_testing_dir = get_package_share_directory('flexbe_testing')

    pkg = DeclareLaunchArgument(
        "pkg",
        default_value="flexbe_states")
    path = DeclareLaunchArgument(
        "path",
        default_value=path_to_test)

    flexbe_testing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(flexbe_testing_dir + "/launch/flexbe_testing.launch"),
        launch_arguments={
            'compact_format': "False",
            'package': LaunchConfiguration("pkg"),
            "testcases": LaunchConfiguration("path") + "/calculation_state_simple.test\n" +
                         LaunchConfiguration("path") + "/calculation_state_none.test\n" +
                         LaunchConfiguration("path") + "/check_condition_state_true.test\n" +
                         LaunchConfiguration("path") + "/check_condition_state_invalid.test\n" +
                         LaunchConfiguration("path") + "/decision_state_simple.test\n" +
                         LaunchConfiguration("path") + "/flexible_calculation_state_simple.test\n" +
                         LaunchConfiguration("path") + "/input_state_import.test\n" +
                         LaunchConfiguration("path") + "/log_state_string.test\n" +
                         LaunchConfiguration("path") + "/log_state_int.test\n" +
                         LaunchConfiguration("path") + "/log_state_msg.test\n" +
                         LaunchConfiguration("path") + "/operator_decision_state_suggested.test\n" +
                         LaunchConfiguration("path") + "/subscriber_state_unavailable.test\n" +
                         LaunchConfiguration("path") + "/subscriber_state_pose.test\n" +
                         LaunchConfiguration("path") + "/wait_state_short.test"

        }.items()
    )

    return (
        launch.LaunchDescription([
            pkg,
            path,
            flexbe_testing
        ]),
        {
            'flexbe_testing': flexbe_testing,
        }
    )
