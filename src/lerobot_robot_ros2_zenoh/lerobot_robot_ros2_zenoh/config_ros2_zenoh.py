"""
Configuration class for ROS2 Zenoh robot integration.
"""
from dataclasses import dataclass, field
from typing import List, Union, Dict

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("ros2_zenoh")
@dataclass
class ROS2ZenohConfig(RobotConfig):
    """
    Configuration for ROS2 Zenoh robot integration.

    Args:
        joint_states_topic: ROS2 topic to subscribe to for joint states (default: "/joint_states")
        joint_trajectory_topic: ROS2 topic(s) to publish joint trajectory commands.
            Can be:
            - A string: Single topic for all joints (default: "/leader/joint_trajectory")
            - A dict: Mapping of joint names to topics, e.g. {"arm_l_joint1": "/left/topic", ...}
        joint_names: List of joint names in the order expected by the robot
        domain_id: ROS2 domain ID (default: 0)
        router_ip: Zenoh router IP address (default: "127.0.0.1")
        router_port: Zenoh router port (default: 7447)
        cameras: Dictionary of camera configurations keyed by camera name
    """
    joint_states_topic: str = "/joint_states"
    joint_trajectory_topic: Union[str, Dict[str, str]] = "/leader/joint_trajectory"
    joint_names: List[str] = field(default_factory=list)
    domain_id: int = 30
    router_ip: str = "127.0.0.1"
    router_port: int = 7447
    cameras: dict[str, CameraConfig] = field(default_factory=dict)