import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rover_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/sensors.yaml"]),
        ("share/" + package_name + "/launch", glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toolbox Robotics",
    maintainer_email="rover@toolboxrobotics.local",
    description="BNO086 (UART-RVC) IMU and L76X GPS drivers.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bno086_rvc_node = rover_sensors.bno086_rvc_node:main",
            "gps_l76x_node = rover_sensors.gps_l76x_node:main",
            "main_camera_node = rover_sensors.main_camera_node:main",
        ],
    },
)
