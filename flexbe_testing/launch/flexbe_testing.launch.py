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
                         "~compact_format": LaunchConfiguration("compact_format")
                         }])
        ])
