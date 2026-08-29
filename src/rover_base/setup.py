from setuptools import find_packages, setup

package_name = "rover_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/base_topology.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toolbox Robotics",
    maintainer_email="rover@toolboxrobotics.local",
    description="Base Mega #1 bridge: 6-wheel drive, 4-corner steering, quadrature encoders, wheel odometry.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "base_bridge_node = rover_base.base_bridge_node:main",
            "odometry_node = rover_base.odometry_node:main",
        ],
    },
)
