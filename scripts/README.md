# Piper Robot Scripts

This directory contains utility scripts for the Piper robot system.

## Scripts

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

## Adding New Scripts

When adding new utility scripts:
1. Include a comprehensive docstring at the top of the script
2. Add command-line argument parsing with help text
3. Update this README with usage instructions
4. Make the script executable: `chmod +x scripts/your_script.py` 