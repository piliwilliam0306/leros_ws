"""
LeRobot ROS2 Zenoh Robot Integration

A LeRobot-compatible robot interface that communicates via ROS2 topics using Zenoh.
"""

from lerobot.robots.config import RobotConfig
from .config_ros2_zenoh import ROS2ZenohConfig
from .ros2_zenoh import ROS2Zenoh
from .config_ros2_camera import ROS2CameraConfig
from .camera_ros2 import ROS2Camera
from .config_ffw_bg2 import FFWBG2FollowerAIConfig

# Import keyboard joint teleoperator to register it
from .keyboard_joint_teleop import KeyboardJointTeleopConfig, KeyboardJointTeleop

# Auto-register the config classes when the package is imported
RobotConfig.register_subclass(ROS2ZenohConfig)

__all__ = [
    "ROS2ZenohConfig",
    "ROS2Zenoh",
    "ROS2CameraConfig",
    "ROS2Camera",
    "FFWBG2FollowerAIConfig",
    "KeyboardJointTeleopConfig",
    "KeyboardJointTeleop",
]
