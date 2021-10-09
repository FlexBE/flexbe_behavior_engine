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
        default_value="flexbe_testing")
    path = DeclareLaunchArgument(
        "path",
        default_value=flexbe_testing_dir + "/test/res")

    flexbe_testing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(flexbe_testing_dir + "/launch/flexbe_testing.launch"),
        launch_arguments={
            'package': LaunchConfiguration("pkg"),
            "testcases": LaunchConfiguration("path") + "/import_only.test\n" +
                         LaunchConfiguration("path") + "/test_add.test\n" +
                         LaunchConfiguration("path") + "/test_add_bagfile.test\n" +
                         LaunchConfiguration("path") + "/sub_unavailable.test\n" +
                         LaunchConfiguration("path") + "/behavior.test\n"

        }.items()
    )

    return (
        launch.LaunchDescription([
            pkg,
            path,
            flexbe_testing
        ])
    )
