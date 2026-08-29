import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rover_web_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["static/*"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Toolbox Robotics",
    maintainer_email="rover@toolboxrobotics.local",
    description="Browser-based ground control console for the rover.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "web_gui_node = rover_web_gui.main:main",
        ],
    },
)
