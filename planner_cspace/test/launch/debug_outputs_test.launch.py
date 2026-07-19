import os
import unittest

import launch_testing
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_test_description():
    set_env = SetEnvironmentVariable("GCOV_PREFIX", "/tmp/gcov/planner_cspace_debug_outputs")
    gtest = Node(
        package="planner_cspace",
        executable="test_debug_outputs",
        name="test_debug_outputs",
        output="screen",
    )
    launch_file = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("planner_cspace"),
                "test",
                "debug_outputs_rostest.test",
            )
        )
    )
    return LaunchDescription([set_env, gtest, launch_file, ReadyToTest()]), {
        "test_node": gtest
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(test_node, timeout=300.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    def test_gtest_pass(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
