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
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_testing.actions import ReadyToTest
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def setup_launch(context: LaunchContext):
    odom_delay = LaunchConfiguration("odom_delay").perform(context)
    use_odom = LaunchConfiguration("use_odom").perform(context)
    use_time_optimal_control = LaunchConfiguration("use_time_optimal_control").perform(
        context
    )

    set_env = SetEnvironmentVariable(
        "GCOV_PREFIX",
        f"/tmp/gcov/trajectory_tracker_d{odom_delay}_{use_odom}_{use_time_optimal_control}",
    )

    return (set_env,)


def generate_test_description():
    args = [
        DeclareLaunchArgument("odom_delay", default_value="0.0"),
        DeclareLaunchArgument("use_odom", default_value="false"),
        DeclareLaunchArgument("use_time_optimal_control", default_value="true"),
    ]
    launch_file = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("trajectory_tracker"),
                "test",
                "trajectory_tracker_rostest.test",
            )
        )
    )
    use_odom = LaunchConfiguration("use_odom")
    gtest = Node(
        package="trajectory_tracker",
        executable="test_trajectory_tracker",
        output="screen",
        ros_arguments=[
            "-r",
            "trajectory_tracker_test:__node:=test_trajectory_tracker",
        ],
        parameters=[
            {"odom_delay": LaunchConfiguration("odom_delay")},
        ],
    )

    return LaunchDescription(
        [
            *args,
            SetParameter("error_lin", 0.03, condition=UnlessCondition(use_odom)),
            SetParameter("error_ang", 0.02, condition=UnlessCondition(use_odom)),
            SetParameter("use_sim_time", "true"),  # clock is provided by the test node
            OpaqueFunction(function=setup_launch),
            launch_file,
            ReadyToTest(),
            gtest,
        ]
    ), {"test_node": gtest}


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(test_node, timeout=300.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    def test_gtest_pass(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
