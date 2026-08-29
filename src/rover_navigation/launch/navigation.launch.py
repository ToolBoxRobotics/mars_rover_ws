"""Launches the full Nav2 stack (map_server, AMCL, planner, controller,
behavior server, bt_navigator, lifecycle management - via nav2_bringup's
own bringup_launch.py rather than composing each node by hand) for
autonomous navigation against a previously saved map.

No map ships with this workspace - build one first with slam.launch.py
(see that file's docstring), save it, then point this at it:

    ros2 launch rover_navigation navigation.launch.py map:=/path/to/your_map.yaml

Assumes the rest of the rover (robot_state_publisher, rover_base
including its odometry node, and the LIDAR) is already up via
rover_bringup's bringup.launch.py; this file only adds Nav2 on top of
that, it does not start the rover itself.

Send goals either from RViz (rover_navigation/rviz/navigation.rviz has
the "2D Pose Estimate" and "Nav2 Goal" tools already added) or from the
command line with `ros2 action send_goal /navigate_to_pose ...`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(get_package_share_directory("rover_navigation"), "config", "nav2_params.yaml")

    map_arg = DeclareLaunchArgument(
        "map",
        description=(
            "Full path to a saved map .yaml file - required, no default, since "
            "silently navigating against the wrong (or a nonexistent) map is "
            "worse than an explicit launch-time error. Build one with "
            "slam.launch.py first if you don't have one yet."
        ),
    )
    params_arg = DeclareLaunchArgument("params_file", default_value=default_params)

    nav2_bringup_launch = os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")

    return LaunchDescription(
        [
            map_arg,
            params_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "params_file": LaunchConfiguration("params_file"),
                    "use_sim_time": "false",
                    "autostart": "true",
                }.items(),
            ),
        ]
    )
