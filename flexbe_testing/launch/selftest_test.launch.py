from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

flexbe_testing_dir = get_package_share_directory('flexbe_testing')
path = flexbe_testing_dir + "/test/res"

testcases  = path + "/import_only.test \n"
testcases += path + "/test_add.test \n"
testcases += path + "/sub_unavailable.test \n"
testcases += path + "/behavior.test \n"

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("pkg", default_value="flexbe_testing"),
        DeclareLaunchArgument("testcases", default_value=testcases),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(flexbe_testing_dir + "/launch/flexbe_testing.launch.py"),
            launch_arguments={
                'package': LaunchConfiguration("pkg"),
                'testcases': LaunchConfiguration("testcases"),
            }.items()
        )
    ])
