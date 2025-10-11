import os
import unittest

import launch_testing
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def setup_launch(context: LaunchContext):
    antialias_start = LaunchConfiguration("antialias_start").perform(context)
    fast_map_update = LaunchConfiguration("fast_map_update").perform(context)
    with_tolerance = LaunchConfiguration("with_tolerance").perform(context)
    enable_crowd_mode = LaunchConfiguration("enable_crowd_mode").perform(context)

    gcov_prefix = f"/tmp/gcov/planner_cspace_navigation_{antialias_start}_{fast_map_update}_{with_tolerance}_{enable_crowd_mode}"
    set_env = SetEnvironmentVariable("GCOV_PREFIX", gcov_prefix)

    return (set_env,)


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
    node_name = PythonExpression(
        [
            "'test_navigate_' + '",
            LaunchConfiguration("antialias_start"),
            "' + '_' + '",
            LaunchConfiguration("fast_map_update"),
            "' + '_' + '",
            LaunchConfiguration("with_tolerance"),
            "' + '_' + '",
            LaunchConfiguration("enable_crowd_mode"),
            "'",
        ]
    )
    gtest = Node(
        package="planner_cspace",
        executable="test_navigate",
        name=node_name,
        output="screen",
        parameters=[{"enable_crowd_mode": LaunchConfiguration("enable_crowd_mode")}],
    )
    setup = OpaqueFunction(function=setup_launch)
    return LaunchDescription([*args, setup, launch_file, gtest, ReadyToTest()]), {
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
