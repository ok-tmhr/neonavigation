from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "GCOV_PREFIX", value="/tmp/gcov/trajectory_tracker_overshoot"
            ),
            ExecuteProcess(
                cmd=[
                    "./build/trajectory_tracker/test/test_trajectory_tracker_overshoot"
                ],
                output="screen",
            ),
            Node(
                package="trajectory_tracker",
                executable="trajectory_tracker",
                output="screen",
                parameters=[
                    {
                        "max_vel": 1.0,
                        "max_acc": 2.0,
                        "max_angvel": 0.5,
                        "max_angacc": 2.0,
                        "goal_tolerance_dist": 0.01,
                        "goal_tolerance_ang": 0.0075,
                        "stop_tolerance_dist": 0.01,
                        "stop_tolerance_ang": 0.005,
                        "look_forward": 0.0,
                        "k_dist": 4.5,
                        "k_ang": 3.0,
                        "k_avel": 4.0,
                        "gain_at_vel": 1.0,
                        "dist_lim": 0.5,
                        "use_odom": True,
                        "odom_timeout_sec": 0.1,
                        "hz": 30.0,
                    }
                ],
            ),
        ]
    )
