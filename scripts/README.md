# Piper Robot Scripts

This directory contains utility scripts for the Piper robot system.

## Scripts

### create_gripper_config.py

A unified calibration and configuration tool for both Piper (ST3215) and Gello (Dynamixel) grippers. This tool replaces the need to manually calibrate grippers and edit code - it creates YAML configuration files that can be loaded at launch time.

#### Features
- Interactive calibration for both gripper types
- Saves calibration to YAML config files
- Supports multiple gripper configurations
- Includes testing mode to verify calibration
- Generates ready-to-use launch commands

#### Quick Usage
```bash
# Full calibration (both grippers)
python3 create_gripper_config.py --config-name my_gripper_setup

# Use the generated config
ros2 launch piper start_piper.launch.py \
    gripper_exist:=true \
    gripper_config:=/path/to/configs/my_gripper_setup.yaml \
    gello_exist:=true
```

See [GRIPPER_CONFIG_README.md](GRIPPER_CONFIG_README.md) for detailed documentation.

### gripper_calibrate.py

An interactive calibration tool for the ST3215 servo gripper. This tool helps you find the optimal open and close positions for your specific gripper hardware.

#### Features
- Interactive real-time control with arrow keys
- Visual feedback of current position
- Test toggle between open/close positions
- Save calibration values with clear instructions

#### Usage

1. **Run the calibration tool inside the Docker container:**
   ```bash
   docker exec -it piper python3 /app/scripts/gripper_calibrate.py
   ```

2. **Controls:**
   - **Arrow Keys/WASD**: Move gripper in small (10 ticks) or large (50 ticks) increments
   - **Space**: Toggle between open and close positions for testing
   - **[**: Set current position as OPEN position
   - **]**: Set current position as CLOSE position
   - **o/c**: Jump to open or close position
   - **h**: Show help menu
   - **q**: Quit and show results

3. **Calibration Process:**
   - Move the gripper to the fully OPEN position (when Gello trigger is released)
   - Press `[` to save this as the open position
   - Move the gripper to the fully CLOSED position (when Gello trigger is pressed)
   - Press `]` to save this as the close position
   - Use `space` to test toggling between positions

4. **Apply the calibration:**
   - The tool will display the exact values to update in `/app/PiperGello/gello/robots/piper_robot.py`
   - Update the `SERVO_OPEN_TICKS` and `SERVO_CLOSE_TICKS` values
   - Restart the robot system

#### Example Output
```
📊 Results:
   Open position:  1592 ticks (Gello trigger released)
   Close position: 842 ticks (Gello trigger pressed)
   Range: 750 ticks
```

### gello_gripper_calibrate.py

Interactive calibration tool for the Dynamixel-based GELLO gripper. Similar to gripper_calibrate.py but for the Gello side.

#### Usage
```bash
python3 gello_gripper_calibrate.py --port /dev/ttyUSB0 --id 7
```

## Adding New Scripts

When adding new utility scripts:
1. Include a comprehensive docstring at the top of the script
2. Add command-line argument parsing with help text
3. Update this README with usage instructions
4. Consider if the functionality should be integrated into create_gripper_config.py 