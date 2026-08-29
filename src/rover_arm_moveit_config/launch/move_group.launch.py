"""Launches MoveIt2's move_group node for the rover arm - the planning
server that RViz's MotionPlanning panel (or any MoveGroupInterface
client) talks to.

Does NOT launch the arm hardware itself (rover_arm_bridge,
odometry, etc. - see rover_bringup) or the trajectory execution
bridge (rover_arm's trajectory_action_server) - both need to already
be running for a planned trajectory to actually reach the physical
arm. This file only starts the planning side.

robot_description is loaded from rover_description's xacro directly
(an absolute path via get_package_share_directory), not auto-discovered
from within this package, since the URDF genuinely lives in a
different package - rover_description is the single source of truth
for the robot model everywhere else in this workspace too.
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
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([move_group_node])
