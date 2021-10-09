from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

flexbe_onboard_dir = get_package_share_directory('flexbe_onboard')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "no_onboard",
            default_value="False"),
        DeclareLaunchArgument(
            "log_enabled",
            default_value="False"),
        DeclareLaunchArgument(
            "log_folder",
            default_value="~/.flexbe_logs"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(flexbe_onboard_dir + "/behavior_onboard.launch.py"),
            launch_arguments={
                'log_enabled': LaunchConfiguration("log_enabled"),
                'log_folder': LaunchConfiguration("log_folder")
            }.items()
        ),
        Node(package="flexbe_mirror", executable="behavior_mirror_sm", name="behavior_mirror"),
        Node(
            package="flexbe_widget", executable="be_action_server", output="screen",
            name="behavior_action_server")
        ])
