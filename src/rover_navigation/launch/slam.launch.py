"""Launches slam_toolbox in online-async mapping mode.

Drive the rover around (Xbox teleop or the web GUI both work fine)
while this runs to build a map. When you're happy with the coverage,
save it:

    ros2 run nav2_map_server map_saver_cli -f ~/mars_rover_ws/src/rover_navigation/maps/my_map

Then switch to navigation.launch.py for autonomous navigation against
that saved map - the two are not run at the same time.

Assumes the rest of the rover (robot_state_publisher, rover_base
including its odometry node, and the LIDAR) is already up via
rover_bringup's bringup.launch.py; this file only adds SLAM on top of
that, it does not start the rover itself.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory("rover_navigation"), "config", "slam_toolbox_params.yaml"
    )
    params_arg = DeclareLaunchArgument("slam_params_file", default_value=default_params)

    slam_toolbox_launch = os.path.join(
        get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py"
    )

    return LaunchDescription(
        [
            params_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch),
                launch_arguments={"slam_params_file": LaunchConfiguration("slam_params_file")}.items(),
            ),
        ]
    )
