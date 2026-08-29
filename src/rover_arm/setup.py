from setuptools import find_packages, setup

package_name = "rover_arm"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/arm_topology.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toolbox Robotics",
    maintainer_email="rover@toolboxrobotics.local",
    description="Arm Mega #2 bridge: 5-axis joint-space control with calibration-switch homing, plus a MoveIt2 trajectory execution bridge.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_bridge_node = rover_arm.arm_bridge_node:main",
            "trajectory_action_server = rover_arm.trajectory_action_server:main",
        ],
    },
)
