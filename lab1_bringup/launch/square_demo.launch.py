"""Starts the whole system, then automatically triggers the square action.

  ros2 launch lab1_bringup square_demo.launch.py side_length:=3.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    system_launch = os.path.join(
        get_package_share_directory("lab1_bringup"), "launch", "system.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("side_length", default_value="3.0"),
        DeclareLaunchArgument("max_linear_speed", default_value="2.0"),
        DeclareLaunchArgument("max_angular_speed", default_value="2.0"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system_launch),
            launch_arguments={
                "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                "max_angular_speed": LaunchConfiguration("max_angular_speed"),
            }.items()),

        # Give the servers a few seconds to come up, then send the square goal.
        TimerAction(period=4.0, actions=[
            ExecuteProcess(
                cmd=["ros2", "action", "send_goal", "/draw_square",
                     "lab1_interfaces/action/DrawSquare",
                     ["{side_length: ", LaunchConfiguration("side_length"), "}"],
                     "--feedback"],
                output="screen"),
        ]),
    ])
