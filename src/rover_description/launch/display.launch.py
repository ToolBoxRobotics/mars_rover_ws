"""Standalone URDF visualization: robot_state_publisher +
joint_state_publisher_gui (sliders for every non-fixed joint, useful
for eyeballing the kinematic chain and joint limits before any real
hardware is connected) + RViz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("rover_description")
    default_xacro = os.path.join(pkg_share, "urdf", "rover.urdf.xacro")
    default_rviz = os.path.join(pkg_share, "rviz", "rover.rviz")

    xacro_arg = DeclareLaunchArgument("model", default_value=default_xacro)

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    return LaunchDescription(
        [
            xacro_arg,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", default_rviz],
                output="screen",
            ),
        ]
    )
