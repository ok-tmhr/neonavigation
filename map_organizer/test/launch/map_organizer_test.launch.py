import os
import unittest

import launch_testing
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest


def generate_test_description():
    arg_tmpfile_prefix = DeclareLaunchArgument(
        "tmpfile_prefix", default_value="/tmp/tmp-map-organizer-988dbe-"
    )
    gtest = Node(
        package="map_organizer",
        executable="test_map_organizer",
        name="test_map_organizer",
        output="screen",
        parameters=[{"file_prefix": LaunchConfiguration("tmpfile_prefix")}],
    )
    launch_file = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("map_organizer"),
                "test",
                "map_organizer_rostest.test",
            )
        ),
        launch_arguments=[("file_prefix", LaunchConfiguration("tmpfile_prefix"))],
    )
    return LaunchDescription([arg_tmpfile_prefix, gtest, launch_file, ReadyToTest()]), {
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
