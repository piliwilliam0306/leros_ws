"""
Configuration class for ROS2 topic-based camera integration.
"""
from dataclasses import dataclass

from lerobot.cameras.configs import CameraConfig, ColorMode, Cv2Rotation


@CameraConfig.register_subclass("ros2")
@dataclass
class ROS2CameraConfig(CameraConfig):
    """Configuration class for ROS2 topic-based cameras.

    This class provides configuration options for cameras that subscribe to
    ROS2 image topics, supporting both compressed and raw image messages.

    Example configurations:
    ```python
    # Compressed image topic
    ROS2CameraConfig(
        topic="/zed/zed_node/left/image_rect_color/compressed",
        fps=30,
        width=640,
        height=480
    )

    # Raw image topic
    ROS2CameraConfig(
        topic="/camera_left/camera_left/color/image_rect_raw",
        fps=30,
        width=1280,
        height=720
    )
    ```

    Attributes:
        topic: ROS2 topic name to subscribe to (e.g., "/camera/image/compressed")
        domain_id: ROS2 domain ID (default: 0)
        router_ip: Zenoh router IP address (default: "127.0.0.1")
        router_port: Zenoh router port (default: 7447)
        fps: Expected frames per second (optional, for metadata)
        width: Expected frame width in pixels (optional, will be detected from messages)
        height: Expected frame height in pixels (optional, will be detected from messages)
        color_mode: Color mode for image output (RGB or BGR). Defaults to RGB.
        rotation: Image rotation setting (0°, 90°, 180°, or 270°). Defaults to no rotation.
        warmup_s: Time reading frames before returning from connect (in seconds)
    """

    topic: str
    domain_id: int = 0
    router_ip: str = "127.0.0.1"
    router_port: int = 7447
    color_mode: ColorMode = ColorMode.RGB
    rotation: Cv2Rotation = Cv2Rotation.NO_ROTATION
    warmup_s: int = 1

    def __post_init__(self) -> None:
        if not self.topic:
            raise ValueError("`topic` must be provided and cannot be empty.")

        if self.color_mode not in (ColorMode.RGB, ColorMode.BGR):
            raise ValueError(
                f"`color_mode` is expected to be {ColorMode.RGB.value} or {ColorMode.BGR.value}, "
                f"but {self.color_mode} is provided."
            )

        if self.rotation not in (
            Cv2Rotation.NO_ROTATION,
            Cv2Rotation.ROTATE_90,
            Cv2Rotation.ROTATE_180,
            Cv2Rotation.ROTATE_270,
        ):
            raise ValueError(
                f"`rotation` is expected to be in "
                f"{(Cv2Rotation.NO_ROTATION, Cv2Rotation.ROTATE_90, Cv2Rotation.ROTATE_180, Cv2Rotation.ROTATE_270)}, "
                f"but {self.rotation} is provided."
            )
