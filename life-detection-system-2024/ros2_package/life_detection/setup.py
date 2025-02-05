import os
from glob import glob

from setuptools import find_packages, setup

package_name = "life_detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Northeastern University Mars Rover Team",
    maintainer_email="northeasternmarsrover@gmail.com",
    description="Life Detection Project",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "carousel_control = life_detection.carousel_control:main",
            "ld_board = life_detection.ld_board:main",
            "life_detection_joy = life_detection.life_detection_joy:main",
            "uv_spec_forwarder = life_detection.uv_spec_forwarder:main",
        ],
    },
)
