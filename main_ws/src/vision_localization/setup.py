# Copyright 2026 Tatsukiyano
from setuptools import find_packages, setup

package_name = "vision_localization"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tatsv",
    maintainer_email="tatsv@example.com",
    description="AprilTag based localization for ROX2026",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tag_localizer = vision_localization.tag_localizer:main",
            "image_syncer = vision_localization.image_syncer:main"
        ],
    },
)
