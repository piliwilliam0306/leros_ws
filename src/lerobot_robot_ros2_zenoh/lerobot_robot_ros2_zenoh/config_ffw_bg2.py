"""
Preset configuration for FFW BG2 Follower AI robot.
"""
from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig

from .config_ros2_camera import ROS2CameraConfig
from .config_ros2_zenoh import ROS2ZenohConfig


# Default joint names for FFW BG2 Follower AI
FFW_BG2_DEFAULT_JOINT_NAMES = [
    "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
    "arm_l_joint5", "arm_l_joint6", "arm_l_joint7",
    "gripper_l_joint1",
    "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
    "arm_r_joint5", "arm_r_joint6", "arm_r_joint7",
    "gripper_r_joint1",
]


def _ffw_bg2_default_cameras() -> dict[str, CameraConfig]:
    """Default camera configuration for FFW BG2."""
    return {
        "cam_head": ROS2CameraConfig(
            topic="/zed/zed_node/left/image_rect_color/compressed",
            fps=30,
            width=640,
            height=480,
        ),
        "cam_wrist_left": ROS2CameraConfig(
            topic="/camera_left/camera_left/color/image_rect_raw/compressed",
            fps=30,
            width=640,
            height=480,
        ),
        "cam_wrist_right": ROS2CameraConfig(
            topic="/camera_right/camera_right/color/image_rect_raw/compressed",
            fps=30,
            width=640,
            height=480,
        ),
    }


@RobotConfig.register_subclass("ffw_bg2_follower_ai")
@dataclass
class FFWBG2FollowerAIConfig(ROS2ZenohConfig):
    """
    Preset configuration for FFW BG2 Follower AI robot.
    
    This configuration includes:
    - 16 joints (7 left arm + 1 left gripper + 7 right arm + 1 right gripper)
    - 3 cameras (head, left wrist, right wrist)
    - Default ROS2 topic settings
    
    You can override any of these defaults when creating the config.
    
    Example:
        ```python
        from lerobot_robot_ros2_zenoh import FFWBG2FollowerAIConfig
        
        # Use all defaults
        config = FFWBG2FollowerAIConfig(id="my_robot")
        
        # Override specific settings
        config = FFWBG2FollowerAIConfig(
            id="my_robot",
            domain_id=0,  # Override domain ID
            # Cameras are already configured, but you can override them:
            cameras={
                "cam_head": ROS2CameraConfig(
                    topic="/custom/camera/topic",
                    fps=60,
                    width=1280,
                    height=720,
                ),
            }
        )
        ```
    """
    
    joint_states_topic: str = "/joint_states"
    joint_trajectory_topic: dict[str, str] = field(default_factory=lambda: {
        # Left arm + left gripper joints
        "arm_l_joint1": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "arm_l_joint2": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "arm_l_joint3": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "arm_l_joint4": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "arm_l_joint5": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "arm_l_joint6": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "arm_l_joint7": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "gripper_l_joint1": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        # Right arm + right gripper joints
        "arm_r_joint1": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "arm_r_joint2": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "arm_r_joint3": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "arm_r_joint4": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "arm_r_joint5": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "arm_r_joint6": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "arm_r_joint7": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "gripper_r_joint1": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
    })
    joint_names: list[str] = field(default_factory=lambda: FFW_BG2_DEFAULT_JOINT_NAMES.copy())
    domain_id: int = 30
    router_ip: str = "127.0.0.1"
    router_port: int = 7447
    cameras: dict[str, CameraConfig] = field(default_factory=_ffw_bg2_default_cameras)
