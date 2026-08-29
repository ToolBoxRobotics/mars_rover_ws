"""Starts the standard ROS 2 `joy` package's joy_node (raw gamepad ->
sensor_msgs/Joy) together with rover_teleop's Xbox 360 mapping node.
No custom joystick driver is written here - this is exactly the kind
of well-trodden hardware support that's better reused than
reimplemented.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("rover_teleop")
    shared_sensitivity_config = os.path.join(pkg_share, "config", "drive_sensitivity.yaml")
    xbox_config = os.path.join(pkg_share, "config", "xbox_teleop.yaml")
    shared_mast_config = os.path.join(
        get_package_share_directory("rover_mast"), "config", "mast_topology.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[{"device_id": 0, "deadzone": 0.05, "autorepeat_rate": 20.0}],
                output="screen",
            ),
            Node(
                package="rover_teleop",
                executable="xbox_teleop_node",
                name="xbox_teleop_node",
                # Order matters only in that later files can override
                # earlier ones for the same key - none of these three
                # overlap by design (see drive_sensitivity.yaml and
                # mast_topology.yaml, both shared with other nodes for
                # exactly this reason).
                parameters=[shared_sensitivity_config, shared_mast_config, xbox_config],
                output="screen",
            ),
        ]
    )
