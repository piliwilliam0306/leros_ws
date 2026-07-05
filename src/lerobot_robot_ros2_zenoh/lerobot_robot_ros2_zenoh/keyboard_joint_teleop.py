"""
Keyboard teleoperator for joint-based robots.

Maps keyboard keys to joint position changes for direct joint control.
"""
import logging
from typing import Any, Dict, Optional
from dataclasses import dataclass, field

from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, PYNPUT_AVAILABLE
from lerobot.teleoperators.keyboard.configuration_keyboard import KeyboardTeleopConfig
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.utils.errors import DeviceNotConnectedError

if PYNPUT_AVAILABLE:
    from pynput import keyboard

logger = logging.getLogger(__name__)


@TeleoperatorConfig.register_subclass("keyboard_joint")
@dataclass
class KeyboardJointTeleopConfig(KeyboardTeleopConfig):
    """Configuration for keyboard joint teleoperator."""

    joint_increment: float = 0.1  # Radians to change per key press
    joint_names: list = field(default_factory=list)  # List of joint names to control


class KeyboardJointTeleop(KeyboardTeleop):
    """
    Keyboard teleoperator for joint-based robots.

    Maps keyboard keys to joint position changes:
    - Number keys 1-7: Control joints 1-7 (increase position)
    - Shift + Number keys: Decrease joint position
    - Q/W/E/R/T/Y/U: Alternative mapping for joints 1-7 (increase)
    - A/S/D/F/G/H/J: Alternative mapping for joints 1-7 (decrease)

    Example:
        ```python
        from lerobot_robot_ros2_zenoh.keyboard_joint_teleop import KeyboardJointTeleop, KeyboardJointTeleopConfig

        config = KeyboardJointTeleopConfig(
            joint_names=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'rh_r1_joint'],
            joint_increment=0.1
        )
        teleop = KeyboardJointTeleop(config)
        teleop.connect()

        while teleop.is_connected:
            action = teleop.get_action()
            robot.send_action(action)
        ```
    """

    config_class = KeyboardJointTeleopConfig
    name = "keyboard_joint"

    def __init__(self, config: KeyboardJointTeleopConfig):
        super().__init__(config)
        self.config = config
        self.current_joint_positions = {}  # Track current positions for delta control
        self.last_action_time = {}  # Track when each joint was last updated
        self._positions_initialized = False  # Track if positions have been initialized from robot state

    def _on_press(self, key):
        """Override to handle both character keys and special keys like Shift."""
        # Handle character keys
        if hasattr(key, "char") and key.char is not None:
            self.event_queue.put((key.char, True))
        elif PYNPUT_AVAILABLE:
            # Handle special keys like Shift
            if key == keyboard.Key.shift or key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
                self.event_queue.put((keyboard.Key.shift, True))

    def _on_release(self, key):
        """Override to handle both character keys and special keys like Shift."""
        # Handle character keys
        if hasattr(key, "char") and key.char is not None:
            self.event_queue.put((key.char, False))
        elif PYNPUT_AVAILABLE:
            # Handle special keys like Shift
            if key == keyboard.Key.shift or key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
                self.event_queue.put((keyboard.Key.shift, False))
        # Handle ESC key
        if PYNPUT_AVAILABLE and key == keyboard.Key.esc:
            logging.info("ESC pressed, disconnecting.")
            self.disconnect()

    @property
    def action_features(self) -> dict:
        """Return action format matching robot's expected format."""
        if self.config.joint_names is None:
            return {}
        return {
            "dtype": "float32",
            "shape": (len(self.config.joint_names),),
            "names": {name: i for i, name in enumerate(self.config.joint_names)},
        }

    def initialize_from_observation(self, observation: Dict[str, Any]) -> None:
        """
        Initialize joint positions from robot's current observation.

        This should be called once at the start of teleoperation to ensure
        the teleoperator starts from the robot's current state rather than zero.

        Args:
            observation: Dictionary with joint positions in format {joint_name.pos: float}
        """
        if not self.config.joint_names:
            return

        for joint_name in self.config.joint_names:
            key = f"{joint_name}.pos"
            if key in observation:
                self.current_joint_positions[joint_name] = float(observation[key])
            else:
                logger.warning(
                    f"Joint '{joint_name}' not found in observation. "
                    f"Available keys: {list(observation.keys())}"
                )
                self.current_joint_positions[joint_name] = 0.0

        self._positions_initialized = True
        logger.info(f"Initialized joint positions from robot observation: {self.current_joint_positions}")

    def get_action(self, observation: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Get joint position actions based on pressed keys.

        If positions haven't been initialized and an observation is provided,
        positions will be initialized from the observation.

        Args:
            observation: Optional robot observation to initialize positions from.
                        If provided and positions aren't initialized, will initialize
                        from current robot state.

        Returns:
            Dictionary with joint positions in format {joint_name.pos: float}
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "KeyboardJointTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

        if self.config.joint_names is None:
            return {}

        self._drain_pressed_keys()

        # Initialize current positions if not set
        if not self._positions_initialized:
            if observation is not None:
                # Initialize from robot's current state
                self.initialize_from_observation(observation)
            else:
                # Fallback to zero initialization (not ideal, but better than error)
                logger.warning(
                    "Joint positions not initialized. Initializing to zero. "
                    "Consider calling initialize_from_observation() or passing observation to get_action()."
                )
                for joint_name in self.config.joint_names:
                    self.current_joint_positions[joint_name] = 0.0
                self._positions_initialized = True

        # Check if Shift is pressed
        shift_pressed = False
        if PYNPUT_AVAILABLE:
            shift_pressed = (keyboard.Key.shift in self.current_pressed and
                           self.current_pressed.get(keyboard.Key.shift, False))

        # Map keys to joint actions
        action = {}

        # Number keys 1-7: Increase joint positions (or decrease with Shift)
        # Q/W/E/R/T/Y/U keys: Alternative increase mapping
        # A/S/D/F/G/H/J keys: Alternative decrease mapping
        for i, joint_name in enumerate(self.config.joint_names):
            if i >= 7:  # Only support 7 joints
                break

            # Number keys (1-7)
            num_key = str(i + 1)

            # Q/W/E/R/T/Y/U keys (alternative mapping for increase)
            alt_keys = ['q', 'w', 'e', 'r', 't', 'y', 'u']
            # A/S/D/F/G/H/J keys (alternative mapping for decrease)
            alt_dec_keys = ['a', 's', 'd', 'f', 'g', 'h', 'j']

            # Check for number key presses
            if num_key in self.current_pressed and self.current_pressed[num_key]:
                if shift_pressed:
                    # Shift + number = decrease
                    self.current_joint_positions[joint_name] -= self.config.joint_increment
                else:
                    # Number alone = increase
                    self.current_joint_positions[joint_name] += self.config.joint_increment

            # Check for alternative increase keys (Q/W/E/R/T/Y/U)
            if i < len(alt_keys) and alt_keys[i] in self.current_pressed and self.current_pressed[alt_keys[i]]:
                self.current_joint_positions[joint_name] += self.config.joint_increment

            # Check for alternative decrease keys (A/S/D/F/G/H/J)
            if i < len(alt_dec_keys) and alt_dec_keys[i] in self.current_pressed and self.current_pressed[alt_dec_keys[i]]:
                self.current_joint_positions[joint_name] -= self.config.joint_increment

            # Add to action dict
            action[f"{joint_name}.pos"] = self.current_joint_positions[joint_name]

        return action

    def reset_joint_positions(self):
        """Reset all joint positions to zero."""
        for joint_name in self.config.joint_names or []:
            self.current_joint_positions[joint_name] = 0.0
