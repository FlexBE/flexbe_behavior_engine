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


"""Interface for setting up pytest framework for colcon testing in ROS 2."""

import time
import unittest
import yaml

from os.path import join

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import MultiThreadedExecutor
from flexbe_testing.tester import Tester
from flexbe_core.proxy import initialize_proxies, shutdown_proxies


class PyTester(unittest.TestCase):
    """
    Pytest unittest wrapper class to run FlexBE tests in unittest framework.

    Used by the colcon test setup

    Derive from this class to define unit test and setUpClass methods
    """

    _test = 0
    _file_path = None
    _package = None
    _tests_folder = None

    def __init__(self, *args, **kwargs):
        """
        Construct instance of unit test class.

        @param package_name - package being tested
        @param tests_folder - location of test files relative to package
        """
        super().__init__(*args, **kwargs)

    @classmethod
    def setUpClass(cls):
        """Must override to define package and tests folder for each test."""
        super().setUpClass()

        package_dir = get_package_share_directory(PyTester._package)
        PyTester._file_path = join(package_dir, PyTester._tests_folder)

    def setUp(self):

        assert PyTester._package is not None, "Must define setUpClass() for particular test"
        assert PyTester._file_path is not None, "Must define setUpClass() for particular test"

        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)

        # Set up ROS
        self.executor = MultiThreadedExecutor(context=self.context)
        self.node = rclpy.create_node(f"flexbe_states_test_{self._test}", context=self.context)
        self.executor.add_node(self.node)
        self.node.declare_parameter("~print_debug_positive", True)
        self.node.declare_parameter("~print_debug_negative", True)
        self.node.declare_parameter("~compact_format", False)

        initialize_proxies(self.node)

        time.sleep(0.1)
        for i in range(5):
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.01)

    def tearDown(self):
        self.node.get_logger().info(f" shutting down {PyTester._package} test {self._test} ...")
        PyTester._test += 1  # increment for next test
        for i in range(10):
            # Allow any lingering pub/sub to clear up
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.01)

        # self.node.get_logger().info("    shutting down proxies in core test %d ... " % (self._test))
        shutdown_proxies()

        # self.node.get_logger().info("    destroy node in core test %d ... " % (self._test))
        self.node.destroy_node()

        self.executor.shutdown()

        # Kill it with fire to make sure not stray published topics are available
        rclpy.shutdown(context=self.context)

    def run_test(self, test_name, timeout_sec=None, max_cnt=50):
        assert PyTester._file_path is not None
        filename = join(PyTester._file_path, test_name + ".test")

        try:
            self.node.get_logger().info(f"  Loading {test_name} from  -->{filename}")
            with open(filename, 'r') as f:
                config = getattr(yaml, 'unsafe_load', yaml.load)(f)
        except IOError as io_error:
            self.node.get_logger().error(f"  Exception in {test_name} : {str(io_error)}")
            self.fail(str(io_error))  # force failure
        except Exception as exc:
            self.node.get_logger().error(f"  Exception in {test_name} : {type(exc)} - {str(exc)}")
            self.fail(str(exc))  # force failure

        self.node.get_logger().error(f"Running test for {test_name}\n config: {config} ...")
        tester = Tester(self.node, self.executor)
        try:
            success = tester.run_pytest(test_name, config, timeout_sec=timeout_sec, max_cnt=max_cnt)
            self.node.get_logger().error(f"    test for {test_name} success={success}")
        except Exception as exc:  # pylint: disable=W0703
            self.node.get_logger().error(f"Exception in test for {test_name}\n  {exc}")
            self.fail(str(exc))  # force failure

        self.assertEqual(success, 1, msg=f"{test_name} failed !")
