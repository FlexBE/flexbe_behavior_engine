from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true') # For simulations

    return LaunchDescription([
        DeclareLaunchArgument("log_enabled", default_value="False"),
        DeclareLaunchArgument("log_folder", default_value="~/.flexbe_logs"),
        DeclareLaunchArgument("log_serialize", default_value="yaml"),
        DeclareLaunchArgument("log_level", default_value="INFO"),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("enable_clear_imports", default_value="False",
            description="Delete behavior-specific module imports after execution."),
        Node(
            name="behavior", package="flexbe_onboard", executable="start_behavior", output="screen",
            parameters=[{"log_enabled": LaunchConfiguration("log_enabled"),
                         "log_folder": LaunchConfiguration("log_folder"),
                         "log_serialize": LaunchConfiguration("log_serialize"),
                         "log_level": LaunchConfiguration("log_level"),
                         "enable_clear_imports": LaunchConfiguration("enable_clear_imports"),
                         "use_sim_time": LaunchConfiguration("use_sim_time")}])
        ])
