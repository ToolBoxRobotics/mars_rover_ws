"""Top-level bringup: robot_state_publisher, all four Arduino bridges,
wheel odometry fused with the IMU via a local EKF (robot_localization)
for a smoother odom -> base_link TF, GPS conversion services
(navsat_transform_node), the two direct-to-USB sensors + main camera,
the RPLIDAR C1, Xbox 360 teleop, the web GUI, and (optionally) SLAM,
Nav2 navigation, or MoveIt2 arm motion planning.

Toggle any subsystem off from the command line, e.g.:
    ros2 launch rover_bringup bringup.launch.py use_teleop:=false use_lidar:=false

SLAM and navigation are mutually exclusive - SLAM builds a map (drive
the rover around while it runs), navigation drives autonomously
against a map already saved from a previous SLAM session. Don't set
both use_slam and use_navigation true at once:
    ros2 launch rover_bringup bringup.launch.py use_slam:=true
    ros2 launch rover_bringup bringup.launch.py use_navigation:=true nav_map:=/path/to/map.yaml

use_moveit:=true adds arm motion planning on top of the always-on
joint-space arm bridge - move_group (rover_arm_moveit_config) plus
rover_arm's trajectory_action_server, which translates MoveIt's
planned trajectories into the same ArmCommand messages manual/teleop
control already uses. Independent of SLAM/navigation; can be combined
with either.
    ros2 launch rover_bringup bringup.launch.py use_moveit:=true

Sensor fusion (rover_navigation/launch/localization.launch.py) is
always on, not optional like SLAM/navigation - it strictly improves
what both of those already consume and never conflicts with either
(see ekf_local_params.yaml's header for why there's no second,
GPS-fusing EKF that would). rover_base's own odometry_node still runs
unconditionally too, but feeds the EKF (`wheel_odom`, no TF of its own)
rather than publishing `odom` directly - see the Node() below.

The four bridge nodes and the odometry node are declared directly here
(each is a single node with no other collaborators) rather than
through a per-package launch file; the multi-node subsystems
(microscope, sensors, teleop, web GUI, SLAM, navigation, localization,
MoveIt) keep their own launch files in their own packages and are
simply included below.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_teleop = DeclareLaunchArgument("use_teleop", default_value="true")
    use_web_gui = DeclareLaunchArgument("use_web_gui", default_value="true")
    use_lidar = DeclareLaunchArgument("use_lidar", default_value="true")
    use_slam = DeclareLaunchArgument(
        "use_slam", default_value="false", description="Build a map with slam_toolbox - see rover_navigation"
    )
    use_navigation = DeclareLaunchArgument(
        "use_navigation", default_value="false", description="Autonomous nav via Nav2 against nav_map"
    )
    nav_map = DeclareLaunchArgument(
        "nav_map", default_value="", description="Map .yaml path, required if use_navigation:=true"
    )
    use_moveit = DeclareLaunchArgument(
        "use_moveit",
        default_value="false",
        description="Arm motion planning (MoveIt2) - move_group plus the trajectory execution bridge",
    )

    base_config = os.path.join(get_package_share_directory("rover_base"), "config", "base_topology.yaml")
    arm_config = os.path.join(get_package_share_directory("rover_arm"), "config", "arm_topology.yaml")
    mast_config = os.path.join(get_package_share_directory("rover_mast"), "config", "mast_topology.yaml")
    antenna_config = os.path.join(
        get_package_share_directory("rover_antenna"), "config", "antenna_topology.yaml"
    )
    power_config = os.path.join(get_package_share_directory("rover_power"), "config", "power_topology.yaml")

    xacro_path = os.path.join(get_package_share_directory("rover_description"), "urdf", "rover.urdf.xacro")
    robot_description = ParameterValue(Command(["xacro ", xacro_path]), value_type=str)

    microscope_launch = os.path.join(
        get_package_share_directory("rover_microscope"), "launch", "microscope.launch.py"
    )
    sensors_launch = os.path.join(get_package_share_directory("rover_sensors"), "launch", "sensors.launch.py")
    teleop_launch = os.path.join(get_package_share_directory("rover_teleop"), "launch", "xbox_teleop.launch.py")
    web_gui_launch = os.path.join(get_package_share_directory("rover_web_gui"), "launch", "web_gui.launch.py")
    slam_launch = os.path.join(get_package_share_directory("rover_navigation"), "launch", "slam.launch.py")
    navigation_launch = os.path.join(
        get_package_share_directory("rover_navigation"), "launch", "navigation.launch.py"
    )
    localization_launch = os.path.join(
        get_package_share_directory("rover_navigation"), "launch", "localization.launch.py"
    )
    move_group_launch = os.path.join(
        get_package_share_directory("rover_arm_moveit_config"), "launch", "move_group.launch.py"
    )

    return LaunchDescription(
        [
            use_teleop,
            use_web_gui,
            use_lidar,
            use_slam,
            use_navigation,
            nav_map,
            use_moveit,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="rover_base",
                executable="base_bridge_node",
                name="rover_base_bridge",
                parameters=[base_config],
                output="screen",
            ),
            Node(
                package="rover_base",
                executable="odometry_node",
                name="rover_odometry_node",
                parameters=[base_config, {"publish_tf": False}],
                # Raw wheel odometry becomes an EKF *input* now rather than
                # the direct source of odom -> base_link: the local EKF
                # (localization.launch.py, included below) fuses this with
                # the IMU and republishes a smoother result as `odom`,
                # which is what actually gets TF-broadcast. See
                # rover_navigation/config/ekf_local_params.yaml for why.
                remappings=[("odom", "wheel_odom")],
                output="screen",
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(localization_launch)),
            Node(
                package="rover_arm",
                executable="arm_bridge_node",
                name="rover_arm_bridge",
                parameters=[arm_config],
                output="screen",
            ),
            # MoveIt2's move_group needs its own full parameter set
            # (SRDF, kinematics, etc.), assembled by rover_arm_moveit_config's
            # own launch file rather than duplicated here - this just
            # includes it. The trajectory_action_server is what
            # actually receives move_group's planned trajectories and
            # turns them into ArmCommand messages the arm bridge above
            # already knows how to execute - see that node's own
            # module docstring for the full reasoning.
            Node(
                package="rover_arm",
                executable="trajectory_action_server",
                name="arm_trajectory_action_server",
                parameters=[arm_config],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_moveit")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(move_group_launch),
                condition=IfCondition(LaunchConfiguration("use_moveit")),
            ),
            Node(
                package="rover_mast",
                executable="mast_bridge_node",
                name="rover_mast_bridge",
                parameters=[mast_config],
                output="screen",
            ),
            Node(
                package="rover_antenna",
                executable="antenna_bridge_node",
                name="rover_antenna_bridge",
                parameters=[antenna_config],
                output="screen",
            ),
            Node(
                package="rover_power",
                executable="power_bridge_node",
                name="rover_power_bridge",
                parameters=[power_config],
                output="screen",
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(microscope_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(sensors_launch)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(teleop_launch),
                condition=IfCondition(LaunchConfiguration("use_teleop")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(web_gui_launch),
                condition=IfCondition(LaunchConfiguration("use_web_gui")),
            ),
            # RPLIDAR C1: reuses the upstream driver rather than
            # reimplementing one. Package/launch-file naming has
            # differed between Slamtec's `sllidar_ros2` GitHub repo and
            # the ROS-index `rplidar_ros` package at various points; the
            # include below targets `rplidar_ros`'s `rplidar_c1_launch.py`
            # (docs.ros.org, Humble). If your install only has
            # sllidar_ros2, swap the package name below and use its
            # `sllidar_c1_launch.py` instead - functionally equivalent.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("rplidar_ros"), "launch", "rplidar_c1_launch.py"
                    )
                ),
                launch_arguments={"serial_port": "/dev/rover/lidar"}.items(),
                condition=IfCondition(LaunchConfiguration("use_lidar")),
            ),
            # SLAM and navigation are mutually exclusive - see this
            # file's own docstring. Neither runs by default.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                condition=IfCondition(LaunchConfiguration("use_slam")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={"map": LaunchConfiguration("nav_map")}.items(),
                condition=IfCondition(LaunchConfiguration("use_navigation")),
            ),
        ]
    )
