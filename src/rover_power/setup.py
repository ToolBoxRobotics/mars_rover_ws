from setuptools import find_packages, setup

package_name = "rover_power"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/power_topology.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toolbox Robotics",
    maintainer_email="rover@toolboxrobotics.local",
    description="Power/environmental monitoring Uno #6 bridge: two batteries' own voltage+current, computer temperature, cooling fan.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "power_bridge_node = rover_power.power_bridge_node:main",
        ],
    },
)
