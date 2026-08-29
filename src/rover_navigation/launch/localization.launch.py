"""Launches the local EKF (fuses wheel odometry + IMU into a smoother
odom -> base_link TF) and navsat_transform_node (GPS <-> map-frame
conversion services and telemetry) together.

Included automatically by rover_bringup's bringup.launch.py - not
meant to be the only thing publishing wheel odometry; it expects
rover_base's odometry_node to already be running with `publish_tf`
false and remapped so its raw output arrives here as `wheel_odom`
(bringup.launch.py sets both), since this EKF becomes the sole
publisher of `odom -> base_link` once it's in the loop.

See ekf_local_params.yaml and navsat_transform_params.yaml for the
full architecture rationale, in particular why there is deliberately
no second, GPS-fusing "global" EKF here (it would conflict with
slam_toolbox/AMCL's ownership of the map -> odom TF).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("rover_navigation")
    ekf_params = os.path.join(pkg_share, "config", "ekf_local_params.yaml")
    navsat_params = os.path.join(pkg_share, "config", "navsat_transform_params.yaml")

    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[ekf_params],
                # /odometry/filtered is robot_localization's own default
                # output topic name; remapped straight to /odom so Nav2's
                # existing odom_topic references (nav2_params.yaml) and
                # slam_toolbox's TF-based consumption both keep working
                # with zero reconfiguration elsewhere.
                remappings=[("odometry/filtered", "odom")],
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                parameters=[navsat_params],
                remappings=[
                    ("imu/data", "/rover_sensors/imu/data"),
                    ("gps/fix", "/rover_sensors/gps/fix"),
                    ("odometry/filtered", "odom"),
                ],
                # odometry/gps (this node's GPS-derived Cartesian output,
                # used for telemetry - see navsat_transform_params.yaml)
                # is left at its own default name; no remap needed.
                output="screen",
            ),
        ]
    )
