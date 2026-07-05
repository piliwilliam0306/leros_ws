"""
ROS2 Zenoh Robot Integration for LeRobot

This robot interface communicates with ROS2 via Zenoh, subscribing to joint states
and publishing joint trajectory commands.
"""
import logging
import threading
import time
from typing import Any, Dict, Optional, Union
import numpy as np

from zenoh_ros2_sdk import ROS2Publisher, ROS2Subscriber, load_message_type, get_message_class

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot

from .config_ros2_zenoh import ROS2ZenohConfig

logger = logging.getLogger(__name__)


class ROS2Zenoh(Robot):
    """
    LeRobot-compatible robot interface for ROS2 communication via Zenoh.

    Subscribes to /joint_states for observations and publishes to
    /arm_controller/joint_trajectory for control commands.
    """

    config_class = ROS2ZenohConfig
    name = "ros2_zenoh"

    def __init__(self, config: ROS2ZenohConfig):
        super().__init__(config)
        self.config = config

        # Validate joint names
        if not self.config.joint_names:
            raise ValueError("joint_names must be provided in config")

        # State storage
        self._latest_joint_state: Optional[Dict[str, float]] = None
        self._state_lock = threading.Lock()
        self._connected = False

        # ROS2 publishers and subscribers (initialized in connect)
        self._joint_state_subscriber: Optional[ROS2Subscriber] = None
        # Support multiple publishers for different joint groups
        self._joint_trajectory_publishers: Dict[str, ROS2Publisher] = {}
        # Mapping of joint names to their topic
        self._joint_to_topic: Dict[str, str] = {}
        self._build_joint_topic_mapping()

        # Initialize cameras from config
        self.cameras = make_cameras_from_configs(config.cameras)

        # Load message types
        self._load_message_types()

    def _build_joint_topic_mapping(self):
        """Build mapping from joint names to trajectory topics."""
        if isinstance(self.config.joint_trajectory_topic, str):
            # Single topic for all joints
            topic = self.config.joint_trajectory_topic
            self._joint_to_topic = {joint: topic for joint in self.config.joint_names}
        elif isinstance(self.config.joint_trajectory_topic, dict):
            # Dict mapping joint names to topics
            self._joint_to_topic = self.config.joint_trajectory_topic.copy()
            # Fill in any missing joints with the first topic found (or raise error)
            if not self._joint_to_topic:
                raise ValueError("joint_trajectory_topic dict cannot be empty")
            default_topic = next(iter(self._joint_to_topic.values()))
            for joint in self.config.joint_names:
                if joint not in self._joint_to_topic:
                    logger.warning(
                        f"Joint '{joint}' not in joint_trajectory_topic mapping, "
                        f"using default topic '{default_topic}'"
                    )
                    self._joint_to_topic[joint] = default_topic
        else:
            raise ValueError(
                f"joint_trajectory_topic must be str or dict, got {type(self.config.joint_trajectory_topic)}"
            )

    def _load_message_types(self):
        """Load required ROS2 message types."""
        # Load JointState message type
        if not load_message_type("sensor_msgs/msg/JointState"):
            raise RuntimeError("Failed to load sensor_msgs/msg/JointState message type")

        # Load JointTrajectory message type
        if not load_message_type("trajectory_msgs/msg/JointTrajectory"):
            raise RuntimeError("Failed to load trajectory_msgs/msg/JointTrajectory message type")

        # Load JointTrajectoryPoint message type (dependency)
        if not load_message_type("trajectory_msgs/msg/JointTrajectoryPoint"):
            raise RuntimeError("Failed to load trajectory_msgs/msg/JointTrajectoryPoint message type")

        # Load builtin_interfaces/Time (dependency)
        if not load_message_type("builtin_interfaces/msg/Time"):
            raise RuntimeError("Failed to load builtin_interfaces/msg/Time message type")

    def _joint_state_callback(self, msg):
        """Callback for joint state messages."""
        try:
            # Create a dictionary mapping joint names to positions
            joint_positions = {}
            if hasattr(msg, 'name') and hasattr(msg, 'position'):
                for name, position in zip(msg.name, msg.position):
                    joint_positions[name] = float(position)

            # Store the latest state
            with self._state_lock:
                self._latest_joint_state = joint_positions
        except Exception as e:
            logger.error(f"Error processing joint state message: {e}", exc_info=True)

    @property
    def _motors_ft(self) -> Dict[str, type]:
        """Motor/joint features."""
        return {f"{joint_name}.pos": float for joint_name in self.config.joint_names}

    @property
    def _cameras_ft(self) -> Dict[str, tuple]:
        """Camera features."""
        return {
            cam_key: (self.config.cameras[cam_key].height, self.config.cameras[cam_key].width, 3)
            for cam_key in self.cameras
        }

    @property
    def observation_features(self) -> Dict[str, type | tuple]:
        """
        Define the observation features structure.

        Returns a dictionary with joint position features and camera features:
        {joint_name.pos: float for each joint in joint_names, camera_name: (height, width, 3) for each camera}
        """
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def action_features(self) -> Dict[str, type]:
        """
        Define the action features structure.

        Returns a dictionary with joint position features in the format:
        {joint_name.pos: float for each joint in joint_names}
        """
        return {f"{joint_name}.pos": float for joint_name in self.config.joint_names}

    @property
    def is_connected(self) -> bool:
        """Check if the robot is connected."""
        cameras_connected = all(cam.is_connected for cam in self.cameras.values()) if self.cameras else True
        return self._connected and cameras_connected

    @property
    def is_calibrated(self) -> bool:
        """
        ROS2 robots typically don't require calibration in the LeRobot sense.
        Always return True.
        """
        return True

    def connect(self, calibrate: bool = True) -> None:
        """
        Connect to the ROS2 robot via Zenoh.

        Args:
            calibrate: Ignored for ROS2 robots (no calibration needed)
        """
        if self._connected:
            logger.warning("Robot is already connected")
            return

        try:
            # Create subscriber for joint states
            self._joint_state_subscriber = ROS2Subscriber(
                topic=self.config.joint_states_topic,
                msg_type="sensor_msgs/msg/JointState",
                callback=self._joint_state_callback,
                domain_id=self.config.domain_id,
                router_ip=self.config.router_ip,
                router_port=self.config.router_port,
            )

            # Create publishers for joint trajectory (one per unique topic)
            unique_topics = set(self._joint_to_topic.values())
            for topic in unique_topics:
                if topic not in self._joint_trajectory_publishers:
                    self._joint_trajectory_publishers[topic] = ROS2Publisher(
                        topic=topic,
                        msg_type="trajectory_msgs/msg/JointTrajectory",
                        domain_id=self.config.domain_id,
                        router_ip=self.config.router_ip,
                        router_port=self.config.router_port,
                    )
                    logger.info(f"Created publisher for trajectory topic: {topic}")

            # Connect cameras
            for cam_key, cam in self.cameras.items():
                try:
                    cam.connect(warmup=True)
                    logger.info(f"Connected camera '{cam_key}' to topic {cam.config.topic}")
                except Exception as e:
                    logger.error(f"Failed to connect camera '{cam_key}': {e}", exc_info=True)
                    raise

            self._connected = True
            logger.info(f"Connected to ROS2 robot via Zenoh (domain_id={self.config.domain_id})")

            # Configure the robot (no-op for ROS2, but required by interface)
            self.configure()

        except Exception as e:
            logger.error(f"Failed to connect to ROS2 robot: {e}", exc_info=True)
            self.disconnect()
            raise

    def disconnect(self) -> None:
        """Disconnect from the ROS2 robot and cleanup resources."""
        if not self._connected:
            return

        try:
            # Disconnect cameras
            for cam_key, cam in self.cameras.items():
                try:
                    cam.disconnect()
                    logger.info(f"Disconnected camera '{cam_key}'")
                except Exception as e:
                    logger.warning(f"Error disconnecting camera '{cam_key}': {e}")

            if self._joint_state_subscriber is not None:
                self._joint_state_subscriber.close()
                self._joint_state_subscriber = None

            # Close all trajectory publishers
            for topic, publisher in self._joint_trajectory_publishers.items():
                try:
                    publisher.close()
                    logger.info(f"Closed publisher for trajectory topic: {topic}")
                except Exception as e:
                    logger.warning(f"Error closing publisher for {topic}: {e}")
            self._joint_trajectory_publishers.clear()

            self._connected = False
            logger.info("Disconnected from ROS2 robot")

        except Exception as e:
            logger.error(f"Error during disconnect: {e}", exc_info=True)
            self._connected = False

    def configure(self) -> None:
        """
        Configure the robot. For ROS2 robots, this is typically a no-op
        as configuration is handled by the ROS2 controller.
        """
        # No configuration needed for ROS2 robots
        pass

    def calibrate(self) -> None:
        """
        Calibrate the robot. For ROS2 robots, calibration is typically
        handled by the ROS2 controller, so this is a no-op.
        """
        # No calibration needed for ROS2 robots
        pass

    def get_observation(self) -> Dict[str, Any]:
        """
        Get the current observation from the robot.

        Returns:
            Dictionary with joint positions in the format:
            {joint_name.pos: float for each joint in joint_names}

        Raises:
            ConnectionError: If the robot is not connected
            RuntimeError: If no joint state has been received after waiting
        """
        if not self._connected:
            raise ConnectionError(f"{self} is not connected.")

        # Wait a bit for the first joint state to arrive (topic discovery)
        max_wait_time = 2.0  # seconds
        wait_interval = 0.1  # seconds
        waited = 0.0

        while waited < max_wait_time:
            with self._state_lock:
                if self._latest_joint_state is not None:
                    break
            time.sleep(wait_interval)
            waited += wait_interval

        with self._state_lock:
            if self._latest_joint_state is None:
                raise RuntimeError(
                    f"No joint state received after {max_wait_time}s. "
                    f"Make sure the joint_states topic ({self.config.joint_states_topic}) is publishing."
                )

            # Extract positions for configured joint names
            observation = {}
            for joint_name in self.config.joint_names:
                if joint_name in self._latest_joint_state:
                    observation[f"{joint_name}.pos"] = self._latest_joint_state[joint_name]
                else:
                    logger.warning(
                        f"Joint '{joint_name}' not found in joint state. "
                        f"Available joints: {list(self._latest_joint_state.keys())}"
                    )
                    # Use 0.0 as default if joint not found
                    observation[f"{joint_name}.pos"] = 0.0

            # Capture images from cameras
            for cam_key, cam in self.cameras.items():
                try:
                    observation[cam_key] = cam.async_read()
                except Exception as e:
                    logger.warning(f"Failed to read from camera '{cam_key}': {e}")
                    # Use zero array as fallback
                    if cam_key in self.config.cameras:
                        cfg = self.config.cameras[cam_key]
                        if cfg.height and cfg.width:
                            observation[cam_key] = np.zeros((cfg.height, cfg.width, 3), dtype=np.uint8)

        return observation

    def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """
        Send an action command to the robot.

        Args:
            action: Dictionary with joint positions in the format:
                   {joint_name.pos: float for each joint in joint_names}

        Returns:
            The action that was sent (potentially modified)

        Raises:
            ConnectionError: If the robot is not connected
        """
        if not self._connected:
            raise ConnectionError(f"{self} is not connected.")

        try:
            # Get message classes for JointTrajectoryPoint, Duration, Header, and Time
            JointTrajectoryPoint = get_message_class("trajectory_msgs/msg/JointTrajectoryPoint")
            Duration = get_message_class("builtin_interfaces/msg/Duration")
            Header = get_message_class("std_msgs/msg/Header")
            Time = get_message_class("builtin_interfaces/msg/Time")

            if JointTrajectoryPoint is None:
                raise RuntimeError("Failed to get message class for JointTrajectoryPoint")
            if Duration is None:
                raise RuntimeError("Failed to get message class for Duration")
            if Header is None:
                raise RuntimeError("Failed to get message class for Header")
            if Time is None:
                raise RuntimeError("Failed to get message class for Time")

            # Create header with timestamp 0,0
            header = Header(
                stamp=Time(sec=0, nanosec=0),
                frame_id=""
            )

            # Create time_from_start (0 seconds, 0 nanoseconds)
            time_from_start = Duration(sec=0, nanosec=0)

            # Group joints by their trajectory topic
            joints_by_topic: Dict[str, list[tuple[str, float]]] = {}
            for joint_name in self.config.joint_names:
                key = f"{joint_name}.pos"
                if key in action:
                    position = float(action[key])
                else:
                    position = 0.0
                
                topic = self._joint_to_topic.get(joint_name)
                if topic is None:
                    logger.warning(f"Joint '{joint_name}' not in topic mapping, skipping")
                    continue
                
                if topic not in joints_by_topic:
                    joints_by_topic[topic] = []
                joints_by_topic[topic].append((joint_name, position))

            # Publish trajectory for each topic
            for topic, joint_data in joints_by_topic.items():
                if topic not in self._joint_trajectory_publishers:
                    logger.error(f"No publisher for topic '{topic}', skipping")
                    continue
                
                # Extract joint names and positions for this topic
                joint_names_for_topic = [name for name, _ in joint_data]
                joint_positions_for_topic = [pos for _, pos in joint_data]

                # Create trajectory point
                trajectory_point = JointTrajectoryPoint(
                    positions=np.array(joint_positions_for_topic, dtype=np.float64),
                    velocities=np.array([], dtype=np.float64),  # Empty for position control
                    accelerations=np.array([], dtype=np.float64),  # Empty for position control
                    effort=np.array([], dtype=np.float64),  # Empty for position control
                    time_from_start=time_from_start
                )

                # Publish the trajectory
                self._joint_trajectory_publishers[topic].publish(
                    header=header,
                    joint_names=joint_names_for_topic,
                    points=[trajectory_point]
                )

            return action

        except Exception as e:
            logger.error(f"Error sending action: {e}", exc_info=True)
            raise
