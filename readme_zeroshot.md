# Zero-shot Quick-Start

This short guide shows how to:

1. Write a **gripper YAML configuration**
2. Launch tele-operation in the common variants
3. Run a few one-liner **debug utilities**

---

## 1. Create a YAML config
Run the interactive calibration script:

```bash
cd /home/zeroshot/piper_ros_docker
python3 scripts/create_gripper_config.py --config-name my_setup
```

The wizard measures open/close limits for Piper (ST3215) and (optionally) the
Gello Dynamixel gripper, then saves **configs/my_setup.yaml** with the correct
structure.  When you only have the Piper gripper or Gello, use `--piper-only`
or `--gello-only`.

Advanced users can still hand-edit the file afterwards.  For reference, the
essential structure is:

```yaml
# configs/my_setup.yaml
piper_gripper:
  device: "/dev/ttyUSB0"       # Serial port of the ST3215 driver
  servo_id: 6                   # Servo ID on the RS-485 bus
  open_ticks: 1592              # <-- auto-filled by the tool
  close_ticks: 900              # <-- auto-filled by the tool
  default_torque: 1000          # Holding torque (PWM) used by controller

# Only required when gello_exist:=true
gello_gripper:
  port: "/dev/ttyUSB0"         # Dynamixel USB bridge (symlink auto-created)
  servo_id: 7                   # Dynamixel ID of the Gello gripper
  open_degrees: 205.0           # <-- auto-filled by the tool
  close_degrees: 145.0          # <-- auto-filled by the tool
  effort_range: [0.5, 3.0]      # Clamp for JointState.effort → current
```

Use the absolute path of the generated file when launching.

### Script options

Key flags of `create_gripper_config.py`:

| Flag | Default | Purpose |
|------|---------|---------|
| `--config-name NAME` | – | Name for **new** YAML (saved to `configs/NAME.yaml`). |
| `--load-config PATH` | – | Load existing YAML and tweak values. |
| `--piper-only` | off | Calibrate only the Piper ST3215 gripper. |
| `--gello-only` | off | Calibrate only the Gello Dynamixel gripper. |
| `--piper-device` | `/dev/ttyACM0` | Serial device of ST3215 USB adaptor. |
| `--piper-id` | `6` | Servo ID on the RS-485 chain. |
| `--piper-torque` | `1000` | Default PWM limit written into the YAML. |
| `--gello-port` | `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NMKV-if00-port0` | USB serial of Dynamixel U2D2. |
| `--gello-id` | `7` | Dynamixel ID of the Gello gripper. |
| `--gello-effort-min` | `0.5` | Lower clamp when mapping effort→current. |
| `--gello-effort-max` | `3.0` | Upper clamp. |
| `--description` | "" | Free-form description stored in YAML metadata. |
| `--robot-serial` | `PIPER_001` | Robot serial number stored in metadata. |

Run with `-h` to see the full help text.

At the end the wizard asks whether to save the file; answer **n** to quit
without writing anything.

---

## 2. Launch tele-op
```
# With Gello bridge and gripper
ros2 launch piper start_piper.launch.py \
    gripper_exist:=true  gello_exist:=true \
    gripper_config:=/app/configs/my_setup.yaml

# With Gello bridge but **no** physical gripper
ros2 launch piper start_piper.launch.py \
    gripper_exist:=false gello_exist:=true \
    gripper_config:=/app/configs/my_setup.yaml

# No Gello at all (arm-only)
ros2 launch piper start_piper.launch.py \
    gello_exist:=false  gripper_exist:=false \
    gripper_config:=/app/configs/my_setup.yaml
```
Nothing else has to be set – the loader publishes every parameter the other nodes need.

---

## 3. Debug utilities
Handy one-liners when something goes wrong:

```bash
# Kill helper scripts / main launch
pkill -f experiments/launch_nodes.py   # ZMQ server (PiperGello)
pkill -f experiments/run_env.py        # Dynamixel bridge
pkill -f "ros2 launch piper start_piper.launch.py"  # Main launch

# List USB serial devices
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# Bring CAN interface up manually (1 Mbit)
sudo ip link set can0 up type can bitrate 1000000
``` 