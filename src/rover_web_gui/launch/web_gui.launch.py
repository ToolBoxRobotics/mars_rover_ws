import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    port_arg = DeclareLaunchArgument("port", default_value="8080")

    # Same file rover_teleop's Xbox controller loads (see its own
    # xbox_teleop.launch.py) - one source of truth for drive
    # sensitivity shared across both control surfaces, via ROS 2's
    # `/**` wildcard node match (see the file itself for why).
    shared_sensitivity_config = os.path.join(
        get_package_share_directory("rover_teleop"), "config", "drive_sensitivity.yaml"
    )
    # Same reasoning, for the mast's max head angles and its "transport
    # position" preset - GET /api/config exposes the latter to the web
    # GUI's own "TRANSPORT POSITION" button.
    shared_mast_config = os.path.join(
        get_package_share_directory("rover_mast"), "config", "mast_topology.yaml"
    )
    # Same reasoning again, for the antenna's azimuth/elevation range -
    # GET /api/config exposes it to the web GUI's antenna sliders.
    shared_antenna_config = os.path.join(
        get_package_share_directory("rover_antenna"), "config", "antenna_topology.yaml"
    )

    return LaunchDescription(
        [
            port_arg,
            Node(
                package="rover_web_gui",
                executable="web_gui_node",
                name="rover_web_gui",
                parameters=[shared_sensitivity_config, shared_mast_config, shared_antenna_config],
                arguments=["--host", "10.0.0.10", "--port", LaunchConfiguration("port")],
                output="screen",
            ),
        ]
    )
