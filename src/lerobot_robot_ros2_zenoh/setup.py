"""
Setup script for lerobot-robot-ros2-zenoh package.
"""
from setuptools import setup, find_packages

setup(
    name="lerobot_robot_ros2_zenoh",
    version="0.1.0",
    description="LeRobot integration for ROS2 robots via Zenoh",
    author="Woojin Wie",
    author_email="wwj@robotis.com",
    packages=find_packages(),
    install_requires=[
        "lerobot",
        "zenoh-ros2-sdk",
    ],
    python_requires=">=3.8",
)
