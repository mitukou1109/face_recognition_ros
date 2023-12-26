import glob
import os

from setuptools import find_packages, setup

package_name = "face_recognition_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (("share/" + package_name + "/launch"), glob.glob("launch/*.[pxy][yma]*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mitsuhiro Sakamoto",
    maintainer_email="mitukou1109@gmail.com",
    description="ROS2 wrapper for face_recognition",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "face_identifier = face_recognition_ros.face_identifier:main"
        ],
    },
)
