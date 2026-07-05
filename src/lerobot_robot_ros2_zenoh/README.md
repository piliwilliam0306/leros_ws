# lerobot-robot-ros2-zenoh

⚠️ **Alpha Software**: This is alpha software and has not been thoroughly tested. Use at your own risk.

LeRobot integration for ROS 2 robots via Zenoh. This package provides a LeRobot-compatible robot interface that communicates with ROS 2 topics using the Zenoh middleware.

## Features

- **Robot Control**: Subscribe to `/joint_states` topic for robot state observations and publish joint trajectory commands to one or multiple topics (supports split controllers for dual-arm robots)
- **Camera Support**: ROS2 topic-based camera support - subscribe to compressed or raw image topics
- **LeRobot Integration**: Full LeRobot compatibility for data collection, training, and inference
- **No ROS2 Required**: Uses Zenoh middleware - no ROS2 installation needed

## Installation

**Important**: This package requires:
1. `zenoh_ros2_sdk` - a local package (not available on PyPI)
2. Local `lerobot` installation (not PyPI version) for plugin discovery to work

```bash
cd /path/to/workspace
git clone https://github.com/ROBOTIS-GIT/zenoh_ros2_sdk.git
git clone https://github.com/ROBOTIS-GIT/lerobot_robot_ros2_zenoh.git
pip install -e zenoh_ros2_sdk
pip install -e lerobot_robot_ros2_zenoh
```

## Configuration

The robot requires the following configuration:

```python
from lerobot_robot_ros2_zenoh import ROS2ZenohConfig

config = ROS2ZenohConfig(
    id="my_robot",
    joint_states_topic="/joint_states",
    joint_trajectory_topic="/arm_controller/joint_trajectory",
    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5"],
    domain_id=30,
    router_ip="127.0.0.1",
    router_port=7447,
)
```

### Configuration Parameters

- `joint_states_topic`: ROS2 topic to subscribe to for joint states (default: `/joint_states`)
- `joint_trajectory_topic`: ROS2 topic(s) to publish joint trajectory commands. Can be:
  - A string: Single topic for all joints (default: `/leader/joint_trajectory`)
  - A dict: Mapping of joint names to topics, e.g. `{"arm_l_joint1": "/left/topic", "arm_r_joint1": "/right/topic"}`. Useful for robots with split controllers (e.g., dual-arm robots)
- `joint_names`: List of joint names in the order expected by the robot (required)
- `domain_id`: ROS2 domain ID (default: 0)
- `router_ip`: Zenoh router IP address (default: `127.0.0.1`)
- `router_port`: Zenoh router port (default: 7447)
- `cameras`: Dictionary of camera configurations (optional, see Camera Configuration below)

### Camera Configuration

You can configure ROS2 topic-based cameras by providing a `cameras` dictionary in the robot config. Each camera subscribes to a ROS2 image topic (compressed or raw).

**Example with cameras:**

```python
from lerobot_robot_ros2_zenoh import ROS2ZenohConfig, ROS2CameraConfig

config = ROS2ZenohConfig(
    id="my_robot",
    joint_names=["joint1", "joint2", "joint3"],
    joint_states_topic="/joint_states",
    joint_trajectory_topic="/arm_controller/joint_trajectory",
    cameras={
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
    },
)
```

**Camera Configuration Parameters:**

- `topic`: ROS2 topic name (required, e.g., `/camera/image/compressed` or `/camera/image_raw`)
- `fps`: Expected frames per second (optional, for metadata)
- `width`: Expected frame width in pixels (optional, will be detected from messages)
- `height`: Expected frame height in pixels (optional, will be detected from messages)
- `domain_id`: ROS2 domain ID (default: same as robot config)
- `router_ip`: Zenoh router IP address (default: same as robot config)
- `router_port`: Zenoh router port (default: same as robot config)
- `color_mode`: Color mode - `ColorMode.RGB` or `ColorMode.BGR` (default: RGB)
- `rotation`: Image rotation - `Cv2Rotation.NO_ROTATION`, `ROTATE_90`, `ROTATE_180`, or `ROTATE_270` (default: NO_ROTATION)

**Supported Image Topics:**

- **Compressed images**: Topics ending with `/compressed` (e.g., `/camera/image/compressed`)
  - Uses `sensor_msgs/msg/CompressedImage` message type
  - Automatically detects JPEG/PNG compression
  - More bandwidth-efficient, recommended for network communication
  
- **Raw images**: Topics without `/compressed` (e.g., `/camera/image_raw`)
  - Uses `sensor_msgs/msg/Image` message type
  - Supports encodings: `rgb8`, `bgr8`, `mono8`, `8UC1`
  - Higher bandwidth usage, but no compression artifacts

## Usage

```python
from lerobot_robot_ros2_zenoh import ROS2Zenoh, ROS2ZenohConfig

# Create configuration
config = ROS2ZenohConfig(
    id="my_robot",
    joint_names=["shoulder", "elbow", "wrist"],
    joint_states_topic="/joint_states",
    joint_trajectory_topic="/arm_controller/joint_trajectory",
)

# Create robot instance
robot = ROS2Zenoh(config)

# Connect to robot
robot.connect()

# Get observation (includes joint positions and camera frames if configured)
observation = robot.get_observation()
print(f"Joint positions: {observation}")
# If cameras are configured:
# observation["cam_head"] contains the camera frame as numpy array
# observation["cam_wrist_left"] contains the camera frame as numpy array
# etc.

# Send action
action = {
    "shoulder.pos": 0.5,
    "elbow.pos": -0.3,
    "wrist.pos": 0.1,
}
robot.send_action(action)

# Disconnect
robot.disconnect()
```

## Command Line Usage

Once installed, you can use this robot with LeRobot CLI tools:

### Teleoperation

Teleoperate the robot using a keyboard or other teleoperation device:

```bash
# Teleoperate with keyboard_joint (recommended for joint-based robots)
lerobot-teleoperate \
    --robot.type=ros2_zenoh \
    --robot.id=my_ros2_robot \
    --robot.joint_names="['joint1','joint2','joint3','joint4','joint5','joint6','rh_r1_joint']" \
    --robot.joint_states_topic=/joint_states \
    --robot.joint_trajectory_topic=/leader/joint_trajectory \
    --robot.domain_id=30 \
    --teleop.type=keyboard_joint \
    --teleop.joint_names="['joint1','joint2','joint3','joint4','joint5','joint6','rh_r1_joint']" \
    --teleop.joint_increment=0.01 \
    --teleop.id=my_keyboard \
    --display_data=false
```

**Keyboard Controls** (for `keyboard_joint` teleop):
- **Number keys 1-7**: Increase position of joints 1-7
- **Shift + Number keys 1-7**: Decrease position of joints 1-7
- **Q/W/E/R/T/Y/U**: Alternative keys to increase joints 1-7
- **A/S/D/F/G/H/J**: Alternative keys to decrease joints 1-7
- **ESC**: Disconnect teleoperator

**Important:** The `joint_names` in the teleop config must match the `joint_names` in the robot config, both in order and naming.

### Recording Data

Record demonstrations while teleoperating:

```bash
lerobot-record \
    --robot.type=ros2_zenoh \
    --robot.id=my_ros2_robot \
    --robot.joint_names="['joint1','joint2','joint3','joint4','joint5','joint6','rh_r1_joint']" \
    --teleop.type=keyboard_joint \
    --teleop.joint_names="['joint1','joint2','joint3','joint4','joint5','joint6','rh_r1_joint']" \
    --teleop.joint_increment=0.01 \
    --teleop.id=my_keyboard \
    --output-dir=./my_dataset
```

## Supported Robots

### Open Manipulator F3M (omy_f3m)

The Open Manipulator F3M robot has 7 joints:
- 6 arm joints: `joint1`, `joint2`, `joint3`, `joint4`, `joint5`, `joint6`
- 1 gripper joint: `rh_r1_joint`

**Important:** When using the `omy_f3m_follower_ai` controller configuration, the gripper joint `rh_r1_joint` is included in the `arm_controller` and can be controlled via the `/leader/joint_trajectory` topic along with all arm joints (similar to `omx_f_follower_ai`).

**Example configuration for omy_f3m:**

```python
from lerobot_robot_ros2_zenoh import ROS2Zenoh, ROS2ZenohConfig

config = ROS2ZenohConfig(
    id="omy_f3m_robot",
    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "rh_r1_joint"],
    joint_states_topic="/joint_states",
    joint_trajectory_topic="/leader/joint_trajectory",
    domain_id=30,
)
```

**Command line usage for omy_f3m:**

```bash
lerobot-teleoperate \
    --robot.type=ros2_zenoh \
    --robot.id=omy_f3m_robot \
    --robot.joint_names="['joint1','joint2','joint3','joint4','joint5','joint6','rh_r1_joint']" \
    --robot.joint_states_topic=/joint_states \
    --robot.joint_trajectory_topic=/leader/joint_trajectory \
    --teleop.type=keyboard_joint \
    --teleop.joint_names="['joint1','joint2','joint3','joint4','joint5','joint6','rh_r1_joint']" \
    --teleop.joint_increment=0.01 \
    --teleop.id=my_keyboard \
    --display_data=false
```

**Note:** This configuration works with the `omy_f3m_follower_ai` setup where all 7 joints (including the gripper) are controlled via a single `joint_trajectory_controller/JointTrajectoryController` publishing to `/leader/joint_trajectory`.

### FFW BG2 Follower AI (ffw_bg2_follower_ai)

The FFW BG2 robot has 16 joints:
- 7 left arm joints: `arm_l_joint1` through `arm_l_joint7`
- 1 left gripper joint: `gripper_l_joint1`
- 7 right arm joints: `arm_r_joint1` through `arm_r_joint7`
- 1 right gripper joint: `gripper_r_joint1`

**✨ Preset Configuration Available!**

A preset configuration is available that includes all joint names and camera settings. You can use it with minimal configuration:

**Simple Python usage (using preset):**

```python
from lerobot_robot_ros2_zenoh import ROS2Zenoh, FFWBG2FollowerAIConfig

# Use preset - all joints and cameras are pre-configured!
config = FFWBG2FollowerAIConfig(id="ffw_bg2_robot")

# Or override specific settings if needed
config = FFWBG2FollowerAIConfig(
    id="ffw_bg2_robot",
    domain_id=0,  # Override domain ID if needed
)

robot = ROS2Zenoh(config)
robot.connect()
```

**Simple command line usage (using preset):**

```bash
# Just specify the robot type - everything else is preset!
lerobot-teleoperate \
    --robot.type=ffw_bg2_follower_ai \
    --robot.id=ffw_bg2_robot \
    --teleop.type=keyboard_joint \
    --teleop.joint_names="['arm_l_joint1','arm_l_joint2','arm_l_joint3','arm_l_joint4','arm_l_joint5','arm_l_joint6','arm_l_joint7','gripper_l_joint1','arm_r_joint1','arm_r_joint2','arm_r_joint3','arm_r_joint4','arm_r_joint5','arm_r_joint6','arm_r_joint7','gripper_r_joint1']" \
    --teleop.joint_increment=0.01 \
    --teleop.id=my_keyboard \
    --display_data=false
```

**Advanced: Manual configuration with split topics (if you need to customize):**

```python
from lerobot_robot_ros2_zenoh import ROS2Zenoh, ROS2ZenohConfig, ROS2CameraConfig

config = ROS2ZenohConfig(
    id="ffw_bg2_robot",
    joint_names=[
        "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
        "arm_l_joint5", "arm_l_joint6", "arm_l_joint7",
        "gripper_l_joint1",
        "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
        "arm_r_joint5", "arm_r_joint6", "arm_r_joint7",
        "gripper_r_joint1",
    ],
    joint_states_topic="/joint_states",
    # Use dict to map joints to different topics
    joint_trajectory_topic={
        # Left arm + left gripper → left broadcaster
        "arm_l_joint1": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "arm_l_joint2": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "arm_l_joint3": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "arm_l_joint4": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "arm_l_joint5": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "arm_l_joint6": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "arm_l_joint7": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        "gripper_l_joint1": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        # Right arm + right gripper → right broadcaster
        "arm_r_joint1": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "arm_r_joint2": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "arm_r_joint3": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "arm_r_joint4": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "arm_r_joint5": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "arm_r_joint6": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "arm_r_joint7": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        "gripper_r_joint1": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
    },
    domain_id=30,
    cameras={
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
    },
)
```

**Note:** The preset configuration works with the `ffw_bg2_follower_ai` setup where:
- Left arm and gripper joints are controlled via `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory`
- Right arm and gripper joints are controlled via `/leader/joint_trajectory_command_broadcaster_right/joint_trajectory`
- All joints are published to `/joint_states` for state observation
- The preset includes:
  - All 16 joint names with correct topic mappings
  - 3 cameras (head, left wrist, right wrist) with default topics
  - Default ROS2 topic settings
  - Default domain ID (30)

## Requirements

- Python 3.8+
- LeRobot (local installation recommended for plugin discovery)
- zenoh_ros2_sdk (local package)
- Zenoh router running (for ROS2 communication)

## Troubleshooting

### Robot type not found

If `lerobot-teleoperate` or other commands don't recognize `ros2_zenoh` as a valid robot type:

1. **Check lerobot version**: Make sure you're using the local lerobot installation, not the PyPI version:
   ```bash
   python -c "import lerobot; print(lerobot.__file__)"
   ```
   Should point to your local lerobot source, not `/usr/local/lib/python3.12/dist-packages/lerobot/`

2. **Reinstall local lerobot**: If PyPI version is installed, uninstall and reinstall local version:
   ```bash
   pip uninstall -y lerobot
   pip install -e /path/to/lerobot
   ```

3. **Verify package installation**: Check that the package is installed:
   ```bash
   pip show lerobot_robot_ros2_zenoh
   ```

4. **Test import**: Verify the package can be imported:
   ```bash
   python -c "import lerobot_robot_ros2_zenoh; from lerobot_robot_ros2_zenoh import ROS2ZenohConfig; print('Package imported successfully')"
   ```

### Multiple trajectory topics not working

If you're using a dict for `joint_trajectory_topic` and joints aren't publishing to the correct topics:

1. **Verify topic mapping**: Ensure all joint names in `joint_names` have corresponding entries in the `joint_trajectory_topic` dict
2. **Check logs**: Look for warnings about joints not in topic mapping
3. **Verify publishers**: Check that publishers are created for all unique topics (see connection logs)