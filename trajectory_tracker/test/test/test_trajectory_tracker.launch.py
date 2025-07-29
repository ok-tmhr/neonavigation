import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetUseSimTime
from launch_testing.actions import ReadyToTest


def setup_launch(context: LaunchContext):
    use_odom = LaunchConfiguration("use_odom").perform(context).lower() in ("true", "1")
    odom_delay = LaunchConfiguration("odom_delay").perform(context)
    if use_odom:
        error_lin = 0.03
    else:
        error_lin = 0.02

    test_trajectory_tracker = ExecuteProcess(
        cmd=[
            "./build/trajectory_tracker/test/test_trajectory_tracker",
            "--ros-args",
            "-p",
            "odom_delay:=" + odom_delay,
            "-p",
            f"error_lin:={error_lin}",
        ],
        name="test_trajectory_tracker",
        output="screen",
    )

    return test_trajectory_tracker,


def generate_test_description():
    # odom_delay = LaunchConfiguration("odom_delay")
    use_odom = LaunchConfiguration("use_odom")
    use_time_optimal_control = LaunchConfiguration("use_time_optimal_control")

    use_sim_time = SetUseSimTime(True)

    # test_trajectory_tracker = (
    #     Node(
    #         package="trajectory_tracker",
    #         executable="test_trajectory_tracker",
    #         parameters=[{"odom_delay": odom_delay, "error_lin": "0.03"}],
    #         condition=IfCondition(use_odom),
    #     ),
    #     Node(
    #         package="trajectory_tracker",
    #         executable="test_trajectory_tracker",
    #         parameters=[{"odom_delay": odom_delay, "error_ang": "0.02"}],
    #         condition=UnlessCondition(use_odom),
    #     ),
    # )

    trajectory_tracker = Node(
        package="trajectory_tracker",
        executable="trajectory_tracker",
        parameters=[
            {
                "max_vel": 1.0,
                "max_acc": 2.0,
                "max_angvel": 0.5,
                "max_angacc": 2.0,
                "goal_tolerance_dist": 0.005,
                "goal_tolerance_ang": 0.005,
                "stop_tolerance_dist": 0.002,
                "stop_tolerance_ang": 0.002,
                "look_forward": 0.0,
                "k_dist": 4.5,
                "k_ang": 3.0,
                "k_avel": 4.0,
                "gain_at_vel": 1.0,
                "dist_lim": 0.5,
                "use_odom": use_odom,
                "odom_timeout_sec": 0.0,
                "use_time_optimal_control": use_time_optimal_control,
                "k_ang_rotation": 8.0,
                "k_avel_rotation": 5.0,
            }.items()
        ],
        output="screen",
    )

    test_trajectory_tracker = OpaqueFunction(function=setup_launch)

    return LaunchDescription(
        [
            DeclareLaunchArgument("odom_delay", default_value="0.0"),
            DeclareLaunchArgument("use_odom", default_value="false"),
            DeclareLaunchArgument("use_time_optimal_control", default_value="true"),
            use_sim_time,
            test_trajectory_tracker,
            trajectory_tracker,
            ReadyToTest(),
        ]
    ), {"test_trajectory_tracker": test_trajectory_tracker}
