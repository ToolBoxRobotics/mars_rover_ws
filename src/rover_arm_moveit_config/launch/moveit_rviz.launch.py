"""Launches RViz with MoveIt's MotionPlanning panel for interactively
planning and executing arm motions - drag the interactive marker at
j5_link, hit Plan, then Execute.

Assumes rover_bringup's bringup.launch.py (robot_state_publisher, the
real arm hardware via rover_arm_bridge and its trajectory action
server) and move_group.launch.py are already running; this file only
adds the RViz visualization on top; it doesn't start move_group itself
or duplicate robot_state_publisher.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    urdf_xacro_path = os.path.join(
        get_package_share_directory("rover_description"), "urdf", "rover.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder("rover_arm", package_name="rover_arm_moveit_config")
        .robot_description(file_path=urdf_xacro_path)
        .robot_description_semantic(file_path="config/rover_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rviz_config = os.path.join(
        get_package_share_directory("rover_arm_moveit_config"), "rviz", "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([rviz_node])
