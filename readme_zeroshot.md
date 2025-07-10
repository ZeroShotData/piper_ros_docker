# Piper Robot - ROS2 Control System

A modern ROS2 control system for the Piper robotic arm with clean architecture and three operation modes.

## Quick Start

### 1. Create Robot Configuration
```bash
cd /home/zeroshot/piper_ros_docker
python3 scripts/create_gripper_config.py --config-name my_setup
```

### 2. Launch Robot (Choose Your Mode)
```bash
# Teleop: Direct control via Gello device (default)
ros2 launch piper piper_unified.launch.py operation_mode:=teleop gripper_config:=/app/configs/baguette.yaml

# Teleop: Keyboard control for testing
ros2 launch piper piper_unified.launch.py operation_mode:=teleop teleop_input:=keyboard gripper_config:=/app/configs/baguette.yaml

# Replay: Remote control via LeRobot/websocket
ros2 launch piper piper_unified.launch.py operation_mode:=replay gripper_config:=/app/configs/baguette.yaml auto_enable:=true

# Monitor: Read-only observation and debugging (lerobot source - default)
ros2 launch piper piper_unified.launch.py operation_mode:=monitor

# Monitor: Hardware monitoring via Gello device
ros2 launch piper piper_unified.launch.py operation_mode:=monitor monitor_source:=gello gripper_config:=/app/configs/baguette.yaml
```

## Operation Modes & Code Examples

### Teleop Mode
**Purpose**: Direct robot control via Gello teleoperation device or keyboard  
**Architecture**: Input Device → Piper Node → Publishes to `/joint_ctrl_single` → Robot  

#### Gello Control (Default)
```bash
# Launch teleop with auto-enable
ros2 launch piper piper_unified.launch.py \
    operation_mode:=teleop \
    gripper_config:=/app/configs/baguette.yaml \
    auto_enable:=true

# Launch teleop without gripper
ros2 launch piper piper_unified.launch.py \
    operation_mode:=teleop \
    gripper_config:=/app/configs/baguette.yaml \
    gripper_exist:=false

# Custom CAN interface
ros2 launch piper piper_unified.launch.py \
    operation_mode:=teleop \
    gripper_config:=/app/configs/baguette.yaml \
    can_port:=can1
```

#### Keyboard Control
```bash
# Terminal 1: Launch robot
ros2 launch piper piper_unified.launch.py operation_mode:=teleop teleop_input:=keyboard gripper_config:=/app/configs/baguette.yaml

# Terminal 2: Run keyboard controller  
ros2 run piper keyboard_joint_teleop
```

**Controls**: `1-7` select joint, `+/-` move by ±0.05, `h` help, `q` quit

### Replay Mode
**Purpose**: Remote control for dataset collection/playback  
**Architecture**: LeRobot → WebSocket → RosBridge → **DIRECTLY** to `/joint_ctrl_single` → Robot  

```bash
# Basic replay mode (auto-starts rosbridge)
ros2 launch piper piper_unified.launch.py \
    operation_mode:=replay \
    gripper_config:=/app/configs/baguette.yaml \
    auto_enable:=true

# Replay without gripper
ros2 launch piper piper_unified.launch.py \
    operation_mode:=replay \
    gripper_config:=/app/configs/baguette.yaml \
    gripper_exist:=false \
    auto_enable:=true

# Custom namespace for multiple arms
ros2 launch piper piper_unified.launch.py \
    operation_mode:=replay \
    gripper_config:=/app/configs/baguette.yaml \
    auto_enable:=true \
    --ros-args -r __ns:=/arm1
```

**LeRobot Integration Example**:
```python
# LeRobot publishes directly to /joint_ctrl_single via rosbridge
# No piper node republishing - eliminates publisher conflicts
import roslibpy

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

joint_pub = roslibpy.Topic(client, '/joint_ctrl_single', 'sensor_msgs/JointState')

# Publish commands directly (piper node only provides feedback)
joint_msg = {
    'name': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper'],
    'position': [0.0, -0.5, 1.0, 0.0, 0.5, 0.0, 0.8]
}
joint_pub.publish(roslibpy.Message(joint_msg))
```

### Monitor Mode  
**Purpose**: Safe monitoring without robot control  
**Sub-modes**: `lerobot` (external) or `gello` (hardware)  

```bash
# LeRobot monitoring (default) - external commands
ros2 launch piper piper_unified.launch.py \
    operation_mode:=monitor \
    monitor_log_format:=json

# Gello hardware monitoring - reads from physical device
ros2 launch piper piper_unified.launch.py \
    operation_mode:=monitor \
    monitor_source:=gello \
    gripper_config:=/app/configs/baguette.yaml

# Data collection pipeline (only outputs when Gello moves)
ros2 launch piper piper_unified.launch.py \
    operation_mode:=monitor \
    monitor_source:=gello \
    gripper_config:=/app/configs/baguette.yaml \
    monitor_log_format:=positions
```

**Monitor Output Examples**:
```bash
# JSON format
{"timestamp": "2025-06-27T20:30:15", "topic": "/joint_ctrl_single", "rate_hz": 100.0, "message": {"positions": [0.12, -0.46, 0.79, 0.01, -0.35, 0.68, 0.8]}}

# Positions format (only when joints move)
0.12 -0.46 0.79 0.01 -0.35 0.68

# Structured format
[MONITOR] 20:30:15.123 | /joint_ctrl_single
  Count: 1245 | Rate: 100.1 Hz | Uptime: 12.4s
  Joints: [0.120, -0.460, 0.790, 0.010, -0.350, 0.680]
```

## Architecture Improvements

### Problem Solved
**Old Architecture** (Broken):
- Replay: LeRobot → WebSocket → Piper → **RE-publishes** to `/joint_ctrl_single` → Robot ❌
- Monitor: LeRobot → WebSocket → Piper → **RE-publishes** + Monitors → **Conflict** ❌

**New Architecture** (Fixed):
- Replay: LeRobot → WebSocket → RosBridge → **DIRECTLY** to `/joint_ctrl_single` → Robot ✅
- Monitor: LeRobot → WebSocket → RosBridge → **DIRECTLY** to `/joint_ctrl_single` → Monitor ✅

### Publisher Strategy
```python
# Mode-specific publisher creation (piper_ctrl_single_node.py:101-109)
if self.operation_mode == 'teleop':
    self._create_teleop_publishers()      # Full set including joint_ctrl_pub
elif self.operation_mode == 'replay':
    self._create_replay_publishers()      # NO joint_ctrl_pub (RosBridge publishes)
elif self.operation_mode == 'monitor':
    self._create_monitor_publishers()     # NO joint_ctrl_pub or servo_cmd_pub
```

## Essential Parameters

```bash
# Required
operation_mode:=teleop|replay|monitor  # Defines node behavior
gripper_config:=/path/to/config.yaml  # Robot parameters

# Common options
can_port:=can0                         # CAN interface
auto_enable:=true|false               # Auto-configured per mode
gripper_exist:=true|false             # Auto-configured per mode

# Teleop-specific
teleop_input:=gello|keyboard          # Input device for teleop mode

# Monitor-specific
monitor_source:=lerobot|gello         # Monitor mode sub-type
monitor_log_format:=json|structured|simple|positions
monitor_rate_interval:=5.0            # Statistics interval
```

## Validation Commands

```bash
# Verify architecture works correctly
ros2 topic info /joint_ctrl_single

# Expected publishers by mode:
# Teleop: 1 publisher (piper node can publish commands)
# Replay: 0 publishers (when only piper running - RosBridge publishes directly)  
# Monitor: 0 publishers (when only piper running - read-only)

# Check message flow
ros2 topic echo /joint_states_single   # Robot feedback
ros2 topic hz /joint_ctrl_single       # Command rate
```

## Troubleshooting

### Publisher Conflicts
```bash
# Error: "already has publisher" 
# Cause: Multiple nodes trying to publish to /joint_ctrl_single
# Solution: Use correct mode

# Wrong: Creates conflict with LeRobot
ros2 launch piper piper_unified.launch.py operation_mode:=teleop

# Correct: No conflict
ros2 launch piper piper_unified.launch.py operation_mode:=replay
```

### Common Issues
```bash
# Check CAN interface
sudo ip link set can0 up type can bitrate 1000000

# Fix USB permissions  
sudo chmod 666 /dev/ttyUSB0 /dev/ttyACM0

# Test rosbridge (replay/monitor modes)
curl -I http://localhost:9090

# Kill conflicting processes
pkill -f piper
```

### Error Messages
The system provides clear guidance:
```
ERROR: Publisher conflict detected!
Topic: /joint_ctrl_single
Current Publishers: 1

Resolution Options:
1. Stop conflicting nodes: ros2 node kill <node_name>
2. Use different namespace: --ros-args -r __ns:=/arm_unique  
3. Switch to replay/monitor mode: operation_mode:=replay
```

## Key Topics

- `/joint_ctrl_single` - Joint commands (Input → Robot)
- `/joint_states_single` - Joint feedback (Output ← Robot)  
- `/end_pose` - End effector pose (Output ← Robot)
- `/arm_status` - Robot status and errors (Output ← Robot)

## Safety Features

- **Monitor Mode**: ALL robot control commands blocked at node level
- **Publisher Validation**: Pre-startup conflict detection prevents runtime issues  
- **Mode Separation**: Clear boundaries prevent accidental cross-mode operation
- **Error Guidance**: Specific resolution steps for common conflicts

**Always use monitor mode for debugging and replay mode for LeRobot integration.**