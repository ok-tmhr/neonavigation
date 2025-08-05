from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
)
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["./build/trajectory_tracker/test/test_trajectory_recorder"],
                output="screen"
            ),
            Node(
                package="trajectory_tracker",
                executable="trajectory_tracker",
                output="screen",
            ),
        ]
    )
