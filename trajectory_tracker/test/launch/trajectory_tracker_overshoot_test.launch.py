import os
import unittest

import launch_testing
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch_testing.actions import ReadyToTest
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_test_description():
    set_env = SetEnvironmentVariable(
        "GCOV_PREFIX", "/tmp/gcov/trajectory_tracker_overshoot"
    )
    use_sim_time = SetParameter("use_sim_time", "false")
    gtest = Node(
        package="trajectory_tracker",
        executable="test_trajectory_tracker_overshoot",
        name="test_trajectory_tracker_overshoot",
        output="screen",
    )
    launch_file = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("trajectory_tracker"),
                "test",
                "trajectory_tracker_overshoot_rostest.test",
            )
        )
    )
    return LaunchDescription(
        [set_env, use_sim_time, gtest, launch_file, ReadyToTest()]
    ), {"test_node": gtest}


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(test_node, timeout=300.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    def test_gtest_pass(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
