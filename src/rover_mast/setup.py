from setuptools import find_packages, setup

package_name = "rover_mast"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/mast_topology.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toolbox Robotics",
    maintainer_email="rover@toolboxrobotics.local",
    description="Mast Uno #3 bridge: pan/tilt head (yaw/pitch calibration homing) plus erect/stow lift.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mast_bridge_node = rover_mast.mast_bridge_node:main",
        ],
    },
)
