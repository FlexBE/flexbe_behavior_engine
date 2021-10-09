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

    TEST_PROC_PATH = os.path.join(path_to_test, 'test_core.py')

    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    test_core = launch.actions.ExecuteProcess(
        cmd=[sys.executable, TEST_PROC_PATH],
        env=proc_env, output='screen'
    )

    return (
        launch.LaunchDescription([
            test_core,
            launch_testing.actions.ReadyToTest()
        ]),
        {
            'test_core': test_core,
        }
    )
