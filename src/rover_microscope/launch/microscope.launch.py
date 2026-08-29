"""Brings up the microscope actuator bridge (Uno #4) and the USB
microscope camera publisher together, since they always belong to the
same physical subsystem at the arm's 3rd wrist joint.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(
        get_package_share_directory("rover_microscope"), "config", "microscope_topology.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="rover_microscope",
                executable="microscope_bridge_node",
                name="rover_microscope_bridge",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="rover_microscope",
                executable="camera_publisher_node",
                name="microscope_camera",
                parameters=[config],
                output="screen",
            ),
        ]
    )
