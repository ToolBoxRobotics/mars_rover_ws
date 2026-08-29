import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rover_microscope"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/microscope_topology.yaml"]),
        ("share/" + package_name + "/launch", glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toolbox Robotics",
    maintainer_email="rover@toolboxrobotics.local",
    description="Microscope Uno #4 bridge plus USB camera publisher (snapshot/recording).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "microscope_bridge_node = rover_microscope.microscope_bridge_node:main",
            "camera_publisher_node = rover_microscope.camera_publisher_node:main",
        ],
    },
)
