import os
import sys
import unittest

import launch_testing
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

watch = {"test_node": ExecuteProcess(cmd=["ls"])}

def setup_launch(context: LaunchContext, ld: LaunchDescription):
    antialias_start = LaunchConfiguration("antialias_start").perform(context)
    fast_map_update = LaunchConfiguration("fast_map_update").perform(context)
    with_tolerance = LaunchConfiguration("with_tolerance").perform(context)
    enable_crowd_mode = LaunchConfiguration("enable_crowd_mode").perform(context)

    gcov_prefix = f"/tmp/gcov/planner_cspace_navigation_{antialias_start}_{fast_map_update}_{with_tolerance}_{enable_crowd_mode}"
    set_env = SetEnvironmentVariable("GCOV_PREFIX", gcov_prefix)

    node_name = f"test_navigate_{antialias_start}_{fast_map_update}_{with_tolerance}_{enable_crowd_mode}"
    gtest = Node(
        package="planner_cspace",
        executable="test_navigate",
        name=node_name,
        output="screen",
        parameters=[{"enable_crowd_mode": LaunchConfiguration("enable_crowd_mode")}],
    )

    ld.add_action(set_env)
    ld.add_action(gtest)
    global watch
    watch["test_node"] = gtest


def generate_test_description():
    args = [
        DeclareLaunchArgument("antialias_start", default_value="false"),
        DeclareLaunchArgument("fast_map_update", default_value="false"),
        DeclareLaunchArgument("with_tolerance", default_value="false"),
        DeclareLaunchArgument("enable_crowd_mode", default_value="false"),
    ]
    launch_file = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("planner_cspace"),
                "test",
                "navigation_rostest.test",
            )
        )
    )
    ld = LaunchDescription([*args, launch_file, ReadyToTest()])
    setup = OpaqueFunction(function=setup_launch, args=[ld])
    ld.add_action(setup)
    global watch
    return ld, watch


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(test_node, timeout=300.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    def test_gtest_pass(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
