# Unified Gripper Configuration Tool

## Overview

The `create_gripper_config.py` script is a unified calibration tool that handles both Piper (ST3215) and Gello (Dynamixel) gripper calibration and saves the results to a YAML configuration file for use with the ROS launch system.

## Features

- **Unified calibration** for both Piper and Gello grippers in one tool
- **Interactive calibration** with real-time feedback
- **Config file management** - create new or update existing configs
- **Testing mode** to verify calibration values
- **Launch command generation** for easy integration

## Usage

### Full Calibration (Both Grippers)
```bash
python3 create_gripper_config.py --config-name my_gripper_setup
```

### Calibrate Only Piper
```bash
python3 create_gripper_config.py --config-name my_setup --piper-only
```

### Calibrate Only Gello
```bash
python3 create_gripper_config.py --config-name my_setup --gello-only
```

### Update Existing Configuration
```bash
python3 create_gripper_config.py --load-config configs/existing.yaml
```

### With Custom Parameters
```bash
python3 create_gripper_config.py --config-name my_setup \
    --piper-device /dev/ttyACM1 \
    --piper-torque 800 \
    --gello-port /dev/serial/by-id/usb-FTDI_... \
    --description "Black gripper with soft tips"
```

## Calibration Controls

### Piper Calibration (ST3215)
- **← / a** : Move -10 ticks (towards open)
- **→ / d** : Move +10 ticks (towards close)
- **↓ / s** : Move -50 ticks (big step open)
- **↑ / w** : Move +50 ticks (big step close)
- **o** : Jump to current OPEN position
- **c** : Jump to current CLOSE position
- **space** : Toggle between open/close
- **[** : Set current as OPEN position
- **]** : Set current as CLOSE position
- **q** : Save and continue

### Gello Calibration (Dynamixel)
- **← / a** : Rotate -2° (towards open)
- **→ / d** : Rotate +2° (towards close)
- **↓ / s** : Rotate -10° (big step open)
- **↑ / w** : Rotate +10° (big step close)
- **o** : Jump to current OPEN position
- **c** : Jump to current CLOSE position
- **space** : Toggle between open/close
- **[** : Set current as OPEN position
- **]** : Set current as CLOSE position
- **q** : Save and continue

## Output Config Format

The tool generates a YAML config file in the `configs/` directory:

```yaml
metadata:
  config_name: "my_gripper_setup"
  created_date: "2024-01-15T10:30:00"
  robot_serial: "PIPER_001"
  description: "Black gripper with soft tips"
  
piper_gripper:
  servo_id: 6
  device: "/dev/ttyACM0"
  open_ticks: 1592
  close_ticks: 842
  default_torque: 1000
  
gello_gripper:
  servo_id: 7
  port: "/dev/serial/by-id/usb-FTDI_..."
  open_degrees: 207.2
  close_degrees: 137.2
  effort_range: [0.5, 3.0]
```

## Using the Configuration

After calibration, the tool displays the exact launch command:

```bash
ros2 launch piper start_piper.launch.py \
    gripper_exist:=true \
    gripper_config:=/path/to/configs/my_gripper_setup.yaml \
    gello_exist:=true
```

## Command Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--config-name` | - | Name for new config file |
| `--load-config` | - | Path to existing config to modify |
| `--piper-only` | False | Only calibrate Piper gripper |
| `--gello-only` | False | Only calibrate Gello gripper |
| `--piper-device` | /dev/ttyACM0 | Piper servo device |
| `--piper-id` | 6 | Piper servo ID |
| `--piper-torque` | 1000 | Default Piper torque limit |
| `--gello-port` | /dev/serial/by-id/... | Gello serial port |
| `--gello-id` | 7 | Gello servo ID |
| `--gello-effort-min` | 0.5 | Minimum effort clamp |
| `--gello-effort-max` | 3.0 | Maximum effort clamp |
| `--description` | "" | Config description |
| `--robot-serial` | PIPER_001 | Robot serial number |

## Troubleshooting

### Permission Errors
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Device Not Found
- Check USB connections
- Verify device paths with `ls /dev/tty*`
- For Gello: `ls /dev/serial/by-id/`

### Servo Not Responding
- Check servo ID matches your hardware
- Verify baud rates (Piper: 1M, Gello: 57600)
- Ensure servo has power

## Example Workflow

1. **Connect both grippers** to your system
2. **Run calibration**:
   ```bash
   python3 create_gripper_config.py --config-name production_grippers
   ```
3. **Calibrate Piper**: Use arrow keys to find optimal open/close positions
4. **Calibrate Gello**: Use arrow keys to find optimal open/close angles
5. **Test the configuration** when prompted
6. **Use the generated launch command** to start your system

## Multiple Gripper Configurations

Create different configs for different gripper setups:

```bash
# Soft-tip grippers
python3 create_gripper_config.py --config-name soft_tips --description "Soft silicone tips"

# Hard grippers for rigid objects
python3 create_gripper_config.py --config-name hard_grippers --description "Hard plastic grippers"

# Wide-opening grippers
python3 create_gripper_config.py --config-name wide_grippers --description "Modified for larger objects"
```

Then use the appropriate config at launch time. 