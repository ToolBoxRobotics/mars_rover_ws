"""Brings up the two sensors that talk straight to USB with no
Arduino in between: the BNO086 IMU (UART-RVC mode) and the L76X GPS.
The RPLIDAR C1 is intentionally NOT included here - see rover_bringup,
which includes the upstream sllidar_ros2 launch file instead of
reimplementing a LIDAR driver from scratch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(get_package_share_directory("rover_sensors"), "config", "sensors.yaml")

    return LaunchDescription(
        [
            Node(
                package="rover_sensors",
                executable="bno086_rvc_node",
                name="bno086_rvc_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="rover_sensors",
                executable="gps_l76x_node",
                name="gps_l76x_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="rover_sensors",
                executable="main_camera_node",
                name="main_camera_node",
                parameters=[config],
                output="screen",
            ),
        ]
    )
