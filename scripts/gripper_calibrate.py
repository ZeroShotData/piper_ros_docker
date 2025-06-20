#!/usr/bin/env python3
"""
ST3215 Gripper Calibration Tool for Piper Robot
===============================================

This tool helps calibrate the ST3215 servo gripper to find the optimal open and close positions.
It provides an interactive interface to test different positions and determine the best range
for your specific gripper hardware.

Usage:
------
1. Run inside the Docker container:
   docker exec -it piper python3 /app/scripts/gripper_calibrate.py

2. Or with custom device/ID:
   docker exec -it piper python3 /app/scripts/gripper_calibrate.py --device /dev/ttyACM0 --id 6

3. Use arrow keys to find optimal positions:
   - Test the fully OPEN position (gripper not pressed on Gello)
   - Test the fully CLOSED position (gripper pressed on Gello)

4. After calibration, update the values in:
   /app/PiperGello/gello/robots/piper_robot.py

   Change these lines:
   SERVO_OPEN_TICKS = 1592   # Your calibrated open value
   SERVO_CLOSE_TICKS = 842   # Your calibrated close value

Author: Piper Robotics
License: MIT
"""

import argparse
import sys
import termios
import tty
import time
import os

try:
    from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    print("ERROR: scservo_sdk not installed")
    print("Please run: pip install feetech-servo-sdk")
    print("Or run this script inside the Piper Docker container")
    sys.exit(1)

# ST3215 servo constants
DEFAULT_SERVO_ID = 6
DEFAULT_DEVICE = "/dev/ttyACM1"  # Common for ST3215 on Piper
BAUDRATE = 1_000_000
PROTOCOL_END = 0  # ST3215 uses protocol 0

# Control registers for ST3215
GOAL_POS_L = 42      # Goal position register
PRESENT_POS_L = 56   # Present position register
TORQUE_ENABLE = 40   # Torque enable register

# Default positions from PiperGello (current calibrated values)
DEFAULT_OPEN = 1592   # Gripper open when Gello trigger not pressed
DEFAULT_CLOSE = 900   # Gripper closed when Gello trigger pressed

# Movement limits
MIN_POSITION = 0
MAX_POSITION = 4095

HELP_MSG = """
╔════════════════════════════════════════════════════════════╗
║           ST3215 Gripper Calibration Tool                  ║
╠════════════════════════════════════════════════════════════╣
║ Movement Controls:                                         ║
║   ← / a : Move gripper -10 ticks (more towards open)       ║
║   → / d : Move gripper +10 ticks (more towards closed)     ║
║   ↓ / s : Move gripper -50 ticks (big step open)           ║
║   ↑ / w : Move gripper +50 ticks (big step close)          ║
║                                                            ║
║ Position Testing:                                          ║
║   o     : Jump to current OPEN position                    ║
║   c     : Jump to current CLOSE position                   ║
║   m     : Go to middle position                            ║
║   space : Toggle between open and close positions          ║
║                                                            ║
║ Calibration:                                               ║
║   [     : Set current position as OPEN                     ║
║   ]     : Set current position as CLOSE                    ║
║                                                            ║
║ Other:                                                     ║
║   r     : Reset to default values                          ║
║   q     : Quit and show results                            ║
║   h/?   : Show this help                                   ║
╚════════════════════════════════════════════════════════════╝

Current defaults:
  Open:  {} ticks (gripper open, Gello trigger released)
  Close: {} ticks (gripper closed, Gello trigger pressed)
""".format(DEFAULT_OPEN, DEFAULT_CLOSE)


class KeyReader:
    """Context manager for reading single keystrokes"""
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self):
        """Read a single key press and handle arrow keys"""
        ch = sys.stdin.read(1)
        # Handle escape sequences (arrow keys)
        if ch == '\x1b':
            ch = sys.stdin.read(1)
            if ch == '[':
                ch = sys.stdin.read(1)
                if ch == 'A': return 'w'  # up arrow
                elif ch == 'B': return 's'  # down arrow
                elif ch == 'C': return 'd'  # right arrow
                elif ch == 'D': return 'a'  # left arrow
        return ch.lower()


def connect_servo(device, servo_id):
    """Connect to the ST3215 servo and return handlers"""
    print(f"\n🔌 Connecting to ST3215 servo...")
    print(f"   Device: {device}")
    print(f"   ID: {servo_id}")
    print(f"   Baud: {BAUDRATE}")
    
    port_handler = PortHandler(device)
    packet_handler = PacketHandler(PROTOCOL_END)
    
    if not port_handler.openPort():
        raise RuntimeError(f"Failed to open port {device}")
    
    if not port_handler.setBaudRate(BAUDRATE):
        port_handler.closePort()
        raise RuntimeError(f"Failed to set baud rate {BAUDRATE}")
    
    # Test connection with ping
    time.sleep(0.1)  # Allow serial to stabilize
    model, res, _ = packet_handler.ping(port_handler, servo_id)
    if res != COMM_SUCCESS:
        port_handler.closePort()
        raise RuntimeError(f"Servo ID {servo_id} not responding on {device}")
    
    print(f"✅ Connected to servo (Model: {model})")
    return port_handler, packet_handler


def main():
    parser = argparse.ArgumentParser(
        description='Interactive calibration tool for ST3215 gripper servo',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with defaults (device=/dev/ttyACM0, id=6)
  python3 gripper_calibrate.py
  
  # Use different device
  python3 gripper_calibrate.py --device /dev/ttyUSB0
  
  # Use different servo ID
  python3 gripper_calibrate.py --id 7
        """
    )
    parser.add_argument('--device', default=DEFAULT_DEVICE,
                        help=f'Serial device path (default: {DEFAULT_DEVICE})')
    parser.add_argument('--id', type=int, default=DEFAULT_SERVO_ID,
                        help=f'Servo ID (default: {DEFAULT_SERVO_ID})')
    args = parser.parse_args()

    try:
        # Connect to servo
        port_handler, packet_handler = connect_servo(args.device, args.id)
        
        # Enable torque
        packet_handler.write1ByteTxRx(port_handler, args.id, TORQUE_ENABLE, 1)
        
        # Get current position
        current_pos, _, _ = packet_handler.read2ByteTxRx(port_handler, args.id, PRESENT_POS_L)
        
        # Initialize calibration values
        test_open = DEFAULT_OPEN
        test_close = DEFAULT_CLOSE
        last_toggle = test_open
        
        print(f"\n📍 Starting position: {current_pos} ticks")
        print("\n" + "="*60)
        print("CALIBRATION TIPS:")
        print("="*60)
        print("1. Press '[' when gripper is at desired OPEN position")
        print("   (Gello trigger released, gripper should be open)")
        print("2. Press ']' when gripper is at desired CLOSE position")
        print("   (Gello trigger pressed, gripper should be closed)")
        print("3. Use SPACE to toggle between positions for testing")
        print("="*60)
        print("\nPress 'h' for help\n")
        
        with KeyReader() as reader:
            while True:
                key = reader.read_key()
                
                if key == 'q':
                    break
                elif key in ('h', '?'):
                    print(HELP_MSG)
                    continue
                
                # Movement commands
                delta = 0
                new_pos = current_pos
                
                if key in ('a', 'A'):  # left arrow - towards open
                    delta = -10
                elif key in ('d', 'D'):  # right arrow - towards close
                    delta = +10
                elif key in ('s', 'S'):  # down arrow - big open
                    delta = -50
                elif key in ('w', 'W'):  # up arrow - big close
                    delta = +50
                elif key == 'o':  # go to open position
                    new_pos = test_open
                    print(f"\n➡️  Testing OPEN position: {test_open} ticks")
                elif key == 'c':  # go to close position
                    new_pos = test_close
                    print(f"\n➡️  Testing CLOSE position: {test_close} ticks")
                elif key == 'm':  # middle position
                    new_pos = (test_open + test_close) // 2
                    print(f"\n➡️  Going to middle: {new_pos} ticks")
                elif key == ' ':  # space - toggle
                    new_pos = test_close if last_toggle == test_open else test_open
                    last_toggle = new_pos
                    print(f"\n🔄 Toggle to {'CLOSE' if new_pos == test_close else 'OPEN'}: {new_pos}")
                elif key == '[':  # set as open position
                    test_open = current_pos
                    print(f"\n✅ Set OPEN position: {test_open} ticks")
                elif key == ']':  # set as close position
                    test_close = current_pos
                    print(f"\n✅ Set CLOSE position: {test_close} ticks")
                elif key == 'r':  # reset to defaults
                    test_open = DEFAULT_OPEN
                    test_close = DEFAULT_CLOSE
                    print(f"\n🔄 Reset to defaults - Open: {test_open}, Close: {test_close}")
                    continue
                else:
                    continue
                
                # Apply movement
                if delta != 0:
                    new_pos = current_pos + delta
                
                # Enforce limits
                new_pos = max(MIN_POSITION, min(MAX_POSITION, new_pos))
                
                # Send command to servo
                result, _ = packet_handler.write2ByteTxRx(port_handler, args.id, GOAL_POS_L, new_pos)
                if result == COMM_SUCCESS:
                    current_pos = new_pos
                    if delta != 0:  # Only show position for incremental moves
                        print(f"Position: {current_pos:4d} (Open:{test_open:4d} Close:{test_close:4d})", end='\r')
                
                time.sleep(0.05)  # Small delay for smooth movement
        
        # Disable torque before closing
        packet_handler.write1ByteTxRx(port_handler, args.id, TORQUE_ENABLE, 0)
        port_handler.closePort()
        
        # Show results
        print("\n\n" + "="*60)
        print("CALIBRATION COMPLETE")
        print("="*60)
        print(f"📊 Results:")
        print(f"   Open position:  {test_open:4d} ticks (Gello trigger released)")
        print(f"   Close position: {test_close:4d} ticks (Gello trigger pressed)")
        print(f"   Range: {abs(test_close - test_open)} ticks")
        print()
        print("📝 To apply these values:")
        print("1. Edit: /app/PiperGello/gello/robots/piper_robot.py")
        print("2. Update these lines:")
        print(f"   SERVO_OPEN_TICKS = {test_open}")
        print(f"   SERVO_CLOSE_TICKS = {test_close}")
        print()
        print("3. Restart the robot:")
        print("   docker exec -it piper bash -c \"pkill -f python3\"")
        print(f"   docker exec -it piper bash -c \"source /opt/ros/humble/setup.bash && \\")
        print("                                   source /app/install/setup.bash && \\")
        print("                                   ros2 launch piper start_piper.launch.py \\")
        print(f"                                   gripper_exist:=true device={args.device}\"")
        print("="*60)
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main()) 