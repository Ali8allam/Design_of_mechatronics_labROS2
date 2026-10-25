"""Starts turtlesim and every Lab 1 server with their parameters.

  ros2 launch lab1_bringup system.launch.py
  ros2 launch lab1_bringup system.launch.py max_linear_speed:=1.0 max_angular_speed:=1.5
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("lab1_bringup"), "config", "turtle_params.yaml")

    params_file = LaunchConfiguration("params_file")
    speed_params = {
        "max_linear_speed": ParameterValue(LaunchConfiguration("max_linear_speed"), value_type=float),
        "max_angular_speed": ParameterValue(LaunchConfiguration("max_angular_speed"), value_type=float),
    }

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="YAML file with controller tuning parameters"),
        DeclareLaunchArgument("max_linear_speed", default_value="2.0",
                              description="Max linear speed of the turtle [units/s]"),
        DeclareLaunchArgument("max_angular_speed", default_value="2.0",
                              description="Max angular speed of the turtle [rad/s]"),

        Node(package="turtlesim", executable="turtlesim_node", name="turtlesim",
             output="screen"),
        Node(package="lab1_turtle_control", executable="go_to_pose_server",
             name="go_to_pose_server", output="screen",
             parameters=[params_file, speed_params]),
        Node(package="lab1_turtle_control", executable="reset_service",
             name="reset_service", output="screen"),
        Node(package="lab1_turtle_control", executable="square_action_server",
             name="square_action_server", output="screen",
             parameters=[params_file]),
    ])
