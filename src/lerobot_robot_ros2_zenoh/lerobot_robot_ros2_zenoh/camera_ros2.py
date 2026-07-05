"""
ROS2 topic-based camera implementation for LeRobot.

This camera subscribes to ROS2 image topics (both compressed and raw) via Zenoh.
"""
import logging
import threading
import time
from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

from zenoh_ros2_sdk import ROS2Subscriber, load_message_type, get_message_class

from lerobot.cameras.camera import Camera
from lerobot.cameras.configs import ColorMode
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.cameras.utils import get_cv2_rotation

from .config_ros2_camera import ROS2CameraConfig

logger = logging.getLogger(__name__)


class ROS2Camera(Camera):
    """
    Camera implementation that subscribes to ROS2 image topics via Zenoh.

    Supports both compressed (sensor_msgs/msg/CompressedImage) and raw
    (sensor_msgs/msg/Image) image messages.

    Example:
        ```python
        from lerobot_robot_ros2_zenoh import ROS2CameraConfig, ROS2Camera

        config = ROS2CameraConfig(
            topic="/zed/zed_node/left/image_rect_color/compressed",
            fps=30,
            width=640,
            height=480
        )
        camera = ROS2Camera(config)
        camera.connect()

        # Read frame synchronously
        frame = camera.read()

        # Read frame asynchronously
        frame = camera.async_read()

        camera.disconnect()
        ```
    """

    def __init__(self, config: ROS2CameraConfig):
        """
        Initialize the ROS2 camera.

        Args:
            config: ROS2 camera configuration
        """
        super().__init__(config)
        self.config = config

        # Determine message type based on topic name
        self.is_compressed = "/compressed" in config.topic
        self.msg_type = (
            "sensor_msgs/msg/CompressedImage" if self.is_compressed else "sensor_msgs/msg/Image"
        )

        # State management
        self._subscriber: ROS2Subscriber | None = None
        self._connected = False
        self._frame_lock = threading.Lock()
        self._latest_frame: NDArray[Any] | None = None
        self._new_frame_event = threading.Event()
        self._frame_received = False

        # Image processing settings
        self.color_mode = config.color_mode
        self.rotation: int | None = get_cv2_rotation(config.rotation)

    def __str__(self) -> str:
        return f"{self.__class__.__name__}({self.config.topic})"

    @property
    def is_connected(self) -> bool:
        """Check if the camera is connected and receiving frames."""
        return self._connected and self._subscriber is not None

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        """
        Find available ROS2 camera topics.

        Note: This is a placeholder implementation. In practice, you would need
        to query ROS2 topics using ros2 topic list or similar methods.
        """
        # TODO: Implement topic discovery via ROS2 CLI or DDS introspection
        return []

    def connect(self, warmup: bool = True) -> None:
        """
        Connect to the ROS2 image topic and start receiving frames.

        Args:
            warmup: If True, waits for at least one frame before returning.
        """
        if self.is_connected:
            logger.warning(f"{self} is already connected.")
            return

        # Load required message types
        if not load_message_type(self.msg_type):
            raise RuntimeError(f"Failed to load message type: {self.msg_type}")

        # Load Header message type (dependency for Image messages)
        if not self.is_compressed:
            if not load_message_type("std_msgs/msg/Header"):
                raise RuntimeError("Failed to load std_msgs/msg/Header message type")

        try:
            # Create subscriber
            self._subscriber = ROS2Subscriber(
                topic=self.config.topic,
                msg_type=self.msg_type,
                callback=self._image_callback,
                domain_id=self.config.domain_id,
                router_ip=self.config.router_ip,
                router_port=self.config.router_port,
            )

            self._connected = True
            logger.info(f"{self} connected to topic {self.config.topic}")

            # Wait for first frame if warmup is enabled
            if warmup:
                start_time = time.time()
                timeout = self.config.warmup_s
                while time.time() - start_time < timeout:
                    if self._frame_received:
                        break
                    time.sleep(0.1)

                if not self._frame_received:
                    logger.warning(
                        f"{self} did not receive a frame during warmup period of {timeout}s. "
                        "Camera may not be publishing yet."
                    )

        except Exception as e:
            logger.error(f"Failed to connect {self}: {e}", exc_info=True)
            self.disconnect()
            raise

    def disconnect(self) -> None:
        """Disconnect from the ROS2 topic and cleanup resources."""
        if not self._connected:
            return

        try:
            if self._subscriber is not None:
                self._subscriber.close()
                self._subscriber = None

            with self._frame_lock:
                self._latest_frame = None
                self._frame_received = False

            self._connected = False
            logger.info(f"{self} disconnected")

        except Exception as e:
            logger.error(f"Error during disconnect of {self}: {e}", exc_info=True)
            self._connected = False

    def _image_callback(self, msg) -> None:
        """
        Callback function called when a new image message is received.

        Args:
            msg: ROS2 image message (CompressedImage or Image)
        """
        try:
            frame = None

            if self.is_compressed:
                # Handle compressed image
                if hasattr(msg, "data"):
                    # Decode compressed image
                    img_array = np.frombuffer(msg.data, np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                else:
                    logger.error(f"{self} CompressedImage message missing 'data' field")
                    return
            else:
                # Handle raw image
                if hasattr(msg, "data") and hasattr(msg, "encoding"):
                    # Decode raw image based on encoding
                    encoding = msg.encoding
                    img_array = np.frombuffer(msg.data, dtype=np.uint8)

                    # Get dimensions from message (required for raw images)
                    if not (hasattr(msg, "height") and hasattr(msg, "width")):
                        logger.error(f"{self} Image message missing height/width fields")
                        return
                    
                    msg_height = msg.height
                    msg_width = msg.width
                    
                    if msg_height <= 0 or msg_width <= 0:
                        logger.error(f"{self} Invalid image dimensions: {msg_width}x{msg_height}")
                        return

                    # Handle different encodings
                    if encoding == "rgb8":
                        frame = img_array.reshape((msg_height, msg_width, 3))
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    elif encoding == "bgr8":
                        frame = img_array.reshape((msg_height, msg_width, 3))
                    elif encoding in ["mono8", "8UC1"]:
                        frame = img_array.reshape((msg_height, msg_width))
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    else:
                        logger.warning(
                            f"{self} Unsupported encoding '{encoding}'. "
                            "Trying to decode as JPEG/PNG."
                        )
                        # Try to decode as compressed image
                        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                        if frame is None:
                            logger.error(f"{self} Failed to decode image with encoding '{encoding}'")
                            return

                    # Update dimensions from message
                    self.height = msg_height
                    self.width = msg_width
                else:
                    logger.error(f"{self} Image message missing required fields")
                    return

            if frame is None:
                logger.warning(f"{self} Failed to decode image")
                return

            # Apply rotation if needed
            if self.rotation is not None:
                frame = cv2.rotate(frame, self.rotation)

            # Convert color mode if needed
            if self.color_mode == ColorMode.RGB:
                # OpenCV uses BGR by default, convert to RGB
                if len(frame.shape) == 3 and frame.shape[2] == 3:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Store the latest frame
            with self._frame_lock:
                self._latest_frame = frame
                self._frame_received = True
                self._new_frame_event.set()

        except Exception as e:
            logger.error(f"{self} Error processing image message: {e}", exc_info=True)

    def read(self, color_mode: ColorMode | None = None) -> NDArray[Any]:
        """
        Read a single frame from the camera synchronously.

        Args:
            color_mode: Desired color mode (overrides config default if provided)

        Returns:
            np.ndarray: Captured frame as a numpy array (height, width, 3)

        Raises:
            DeviceNotConnectedError: If camera is not connected
            TimeoutError: If no frame is received within timeout
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Wait for a new frame with timeout
        timeout = 2.0  # seconds
        if not self._new_frame_event.wait(timeout):
            raise TimeoutError(f"{self} timeout waiting for frame after {timeout}s")

        with self._frame_lock:
            if self._latest_frame is None:
                raise RuntimeError(f"{self} no frame available")

            frame = self._latest_frame.copy()
            self._new_frame_event.clear()

        # Apply color mode conversion if requested
        if color_mode is not None and color_mode != self.color_mode:
            if color_mode == ColorMode.RGB and self.color_mode == ColorMode.BGR:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            elif color_mode == ColorMode.BGR and self.color_mode == ColorMode.RGB:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        return frame

    def async_read(self, timeout_ms: float = 2000.0) -> NDArray[Any]:
        """
        Asynchronously read a single frame from the camera.

        Args:
            timeout_ms: Maximum time to wait for a frame in milliseconds

        Returns:
            np.ndarray: Captured frame as a numpy array (height, width, 3)

        Raises:
            DeviceNotConnectedError: If camera is not connected
            TimeoutError: If no frame is received within timeout
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Wait for a new frame with timeout
        timeout_s = timeout_ms / 1000.0
        if not self._new_frame_event.wait(timeout_s):
            raise TimeoutError(f"{self} timeout waiting for frame after {timeout_ms}ms")

        with self._frame_lock:
            if self._latest_frame is None:
                raise RuntimeError(f"{self} no frame available")

            frame = self._latest_frame.copy()
            self._new_frame_event.clear()

        return frame
