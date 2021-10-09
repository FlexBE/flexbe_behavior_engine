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
    testcases = DeclareLaunchArgument(
        "testcases",
        default_value="")
    time_limit = DeclareLaunchArgument(
        "time-limit",
        default_value="60")
    package = DeclareLaunchArgument(
        "package",
        default_value="")

    path_to_test = os.path.dirname(__file__)
    TEST_PROC_PATH = os.path.join(path_to_test, 'test_proxies.py')

    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    test_proxies = launch.actions.ExecuteProcess(
        cmd=[sys.executable, TEST_PROC_PATH],
        env=proc_env,
        output='screen',
        sigterm_timeout=launch.substitutions.LaunchConfiguration('sigterm_timeout', default=60),
        sigkill_timeout=launch.substitutions.LaunchConfiguration('sigkill_timeout', default=60)
    )

    return (
        launch.LaunchDescription([
            test_proxies,
            launch_testing.actions.ReadyToTest()
        ]),
        {
            'test_proxies': test_proxies,
        }
    )
