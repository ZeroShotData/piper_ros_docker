# Dual-Arm Piper ROS Planning Notes

## 1. High-Level Architecture Diagram

```
┌───────────────────────────────────────────────────────────────────┐
│                          ROS2 System                              │
├───────────────────────────────────────────────────────────────────┤
│  Namespace: /choupette           │  Namespace: /baguette         │
│  ┌─────────────────────────────┐   │  ┌─────────────────────────┐ │
│  │   Choupette Arm Controller  │   │  │  Baguette Arm Controller│ │
│  │  piper_ctrl_single_node     │   │  │  piper_ctrl_single_node │ │
│  └─────────────────────────────┘   │  └─────────────────────────┘ │
│               │                    │               │              │
│  Topics:      │                    │  Topics:      │              │
│  • /choupette/joint_ctrl_single    │  • /baguette/joint_ctrl_single│
│  • /choupette/joint_states         │  • /baguette/joint_states    │
│  • /choupette/arm_status           │  • /baguette/arm_status      │
│  • /choupette/servo/command_raw    │  • /baguette/servo/command_raw│
│               │                    │               │              │
│  ┌─────────────────────────────┐   │  ┌─────────────────────────┐ │
│  │    Choupette Gello Agent    │   │  │   Baguette Gello Agent  │ │
│  │       (USB Device 1)        │   │  │       (USB Device 2)    │ │
│  └─────────────────────────────┘   │  └─────────────────────────┘ │
│               │                    │               │              │
│  ┌─────────────────────────────┐   │  ┌─────────────────────────┐ │
│  │     Choupette Hardware      │   │  │    Baguette Hardware    │ │
│  │  CAN0: Piper Arm Motors     │   │  │  CAN1: Piper Arm Motors │ │
│  │  Serial: Gello Gripper      │   │  │  Serial: Gello Gripper  │ │
│  └─────────────────────────────┘   │  └─────────────────────────┘ │
└───────────────────────────────────────────────────────────────────┘
│                    Shared Components                              │
│  • Single Rosbridge WebSocket Server                              │
│  • Dual Robot State Publishers (choupette_*, baguette_* frames)   │
│  • Global TF Tree with prefixed frames                            │
└───────────────────────────────────────────────────────────────────┘
```

## 2. Precise File/Line Changes Required

### Core Launch File
- **`src/piper/launch/piper_unified.launch.py`** - Add multi-arm support with `arms_config` parameter
- **`src/piper/launch/piper_unified.launch.py`** - Replace hardcoded `--gello_port=` line with per-arm configuration
- **`src/piper/launch/piper_unified.launch.py`** - Guard `('joint_states_single', 'joint_states')` remap with `IfCondition(absent(arms_config))`
- **`src/piper/launch/piper_unified.launch.py`** - Duplicate loader_node and servo_node for each arm namespace (loader first, servo second)

### Configuration Files  
- **`configs/choupette.yaml`** - Create config for choupette (left arm) with servo IDs 4,7
- **`configs/baguette.yaml`** - Update existing baguette config with servo IDs 5,8
- **`configs/dual_arms.yaml`** - Multi-arm configuration defining both arms
- **`gripper_config_loader`** - Update to accept arms_config YAML schema (may need small parsing tweak)

### URDF/Robot Description
- **`src/piper_description/urdf/piper_description.xacro:1-200`** - Convert to macro with `prefix` parameter for frame naming
- **`src/piper_description/urdf/piper_no_gripper_description.xacro:1-150`** - Same macro conversion

### Gello Integration
- **`PiperGello/experiments/run_env.py:296`** - Accept `--gello_port` parameter instead of hardcoded path

## 3. New Configuration & Launch Examples

### Multi-Arm Config: `configs/dual_arms.yaml`
```yaml
arms:
  choupette:
    can_port: "can0"
    joint_topic: "/choupette/joint_ctrl_single"
    joint_state_topic: "/choupette/joint_states_single"
    enable_topic: "/choupette/enable_flag"
    servo_topic: "/choupette/servo/command_raw"
    gello_gripper:
      servo_id: 7  # Physical gripper must be reflashed to ID 7
      port: "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NMED-if00-port0"
      open_degrees: 0
      close_degrees: 30
    piper_gripper:
      servo_id: 4  # Physical gripper must be reflashed to ID 4
      device: /dev/ttyACM0
      open_ticks: 1666
      close_ticks: 916
      default_torque: 1000
      
  baguette:
    can_port: "can1"
    joint_topic: "/baguette/joint_ctrl_single"
    joint_state_topic: "/baguette/joint_states_single"
    enable_topic: "/baguette/enable_flag"
    servo_topic: "/baguette/servo/command_raw"
    gello_gripper:
      servo_id: 8  # Physical gripper must be reflashed to ID 8
      port: "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NMKV-if00-port0"
      open_degrees: 0
      close_degrees: 30
    piper_gripper:
      servo_id: 5  # Physical gripper must be reflashed to ID 5
      device: /dev/ttyACM1
      open_ticks: 1666
      close_ticks: 916
      default_torque: 1000
```

### Single Launch Command
```bash
# Single terminal - Both arms
ros2 launch piper piper_unified.launch.py \
  operation_mode:=teleop \
  arms_config:=/app/configs/dual_arms.yaml \
  --ros-args --log-level warn

# Single arm (backward compatible - requires gripper_config)
ros2 launch piper piper_unified.launch.py \
  operation_mode:=teleop \
  gripper_config:=/app/configs/baguette.yaml
```

## 4. Migration Plan (4-6 hours)

### Phase 1: Launch File Updates (1-2 hours)
1. **Add arms_config parameter** to `piper_unified.launch.py` for multi-arm support
2. **Add logic to iterate over arms** and create namespaced nodes for each
3. **Guard joint_states remap** with `IfCondition(absent(arms_config))` to preserve single-arm behavior
4. **Duplicate loader_node and servo_node** for each arm namespace (loader first, servo second)
5. **Test single-arm compatibility** with all three modes (must pass gripper_config)

### Phase 2: URDF Conversion (2-3 hours)  
4. **Convert URDF to macro** with `prefix` parameter
5. **Create dual URDF** instantiating macro twice with `choupette_`/`baguette_` prefixes
6. **Test visualization** in RViz

### Phase 3: Configuration Setup (30 min)
7. **Create dual configs** for left/right arms with unique servo IDs
8. **Set up hardware** with separate CAN interfaces and USB paths

### Phase 4: Integration Testing (1-2 hours)
9. **Launch dual-arm system** with single command using `arms_config` parameter
10. **Verify topic isolation** and TF frame prefixing
11. **Test all operation modes** (teleop, replay, monitor) with dual arms

## 5. Critical Risks & Mitigations

### Must Fix
- **TF frame collisions** - Convert URDF to macro with prefixing (blocks dual-arm operation)
- **Servo ID conflicts** - Use unique IDs: choupette (4,7), baguette (5,8)
- **Physical gripper reflashing** - Dynamixel servos must be reflashed to new IDs before use
- **Joint states remapping** - Guard with `IfCondition(absent(arms_config))` to preserve single-arm behavior

### Hardware Requirements  
- **Separate CAN interfaces** - `can0`/`can1` prevent motor ID conflicts
- **Stable USB paths** - Use `/dev/serial/by-id/` not `/dev/ttyUSB*`
- **Unique Gello devices** - Two different USB serial converters

### Backward Compatibility
- **Single-arm modes preserved** - All three operation modes (teleop/replay/monitor) continue working
- **Launch parameter defaults** - Existing behavior maintained when `arms_config` not provided
- **Config file compatibility** - Original YAML formats still supported
- **Critical**: Single-arm launches still require `gripper_config` parameter