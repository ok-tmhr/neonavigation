import os
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
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetUseSimTime
from launch_testing.actions import ReadyToTest

watch = {"test_node": ExecuteProcess(cmd=["ls"])}


def setup_launch(context: LaunchContext):
    odom_delay = LaunchConfiguration("odom_delay")
    var_odom_delay = odom_delay.perform(context)
    use_odom = LaunchConfiguration("use_odom").perform(context)
    use_time_optimal_control = LaunchConfiguration("use_time_optimal_control").perform(
        context
    )

    set_env = SetEnvironmentVariable(
        "GCOV_PREFIX",
        f"/tmp/gcov/trajectory_tracker_d{var_odom_delay}_{use_odom}_{use_time_optimal_control}",
    )

    if use_odom.lower() in ("1", "true"):
        gtest = Node(
            package="trajectory_tracker",
            executable="test_trajectory_tracker",
            output="screen",
            ros_arguments=[
                "-r",
                "trajectory_tracker_test:__node:=test_trajectory_tracker",
            ],
            parameters=[{"odom_delay": odom_delay}],
        )
    else:
        gtest = Node(
            package="trajectory_tracker",
            executable="test_trajectory_tracker",
            output="screen",
            ros_arguments=[
                "-r",
                "trajectory_tracker_test:__node:=test_trajectory_tracker",
            ],
            parameters=[
                {
                    "odom_delay": odom_delay,
                    "error_lin": 0.03,
                    "error_ang": 0.02,
                }
            ],
        )

    global watch
    watch["test_node"] = gtest

    return set_env,


def generate_test_description():
    args = [
        DeclareLaunchArgument("odom_delay", default_value="0.0"),
        DeclareLaunchArgument("use_odom", default_value="false"),
        DeclareLaunchArgument("use_time_optimal_control", default_value="true"),
    ]
    launch_file = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("trajectory_tracker"),
                "test",
                "trajectory_tracker_rostest.test",
            )
        )
    )
    global watch
    return LaunchDescription(
        [
            *args,
            OpaqueFunction(function=setup_launch),
            SetUseSimTime(True), # clock is provided by the test node
            launch_file,
            ReadyToTest(),
        ]
    ), watch


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(test_node, timeout=300.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    def test_gtest_pass(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
