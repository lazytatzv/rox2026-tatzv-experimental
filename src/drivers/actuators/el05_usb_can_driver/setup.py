from glob import glob
from setuptools import find_packages, setup

package_name = "el05_usb_can_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maintainer",
    maintainer_email="maintainer@example.com",
    description="ROS 2 driver node for RobStride EL05 motors through Seeed USB-CAN.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "el05_motor_node = el05_usb_can_driver.el05_motor_node:main",
        ],
    },
)
