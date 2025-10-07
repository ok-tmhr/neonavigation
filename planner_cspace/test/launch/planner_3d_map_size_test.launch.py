import os
import unittest

import launch_testing
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
)
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest


def generate_test_description():
    set_env = SetEnvironmentVariable("GCOV_PREFIX", "/tmp/gcov/planner_cspace_map_size")
    gtest = Node(
        package="planner_cspace",
        executable="test_planner_3d_map_size",
        name="test_planner_3d_map_size",
        output="screen",
    )
    launch_file = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("planner_cspace"),
                "test",
                "planner_3d_map_size_rostest.test",
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
