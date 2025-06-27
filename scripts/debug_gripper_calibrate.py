#!/usr/bin/env python3
"""Debug version of gripper calibration with verbose output."""
import time
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# Configuration
SERVO_ID = 6
DEVICE = "/dev/ttyACM0"
BAUDRATE = 1_000_000
PROTOCOL_END = 0

# Registers
GOAL_POS_L = 42
PRESENT_POS_L = 56
TORQUE_ENABLE = 40

def main():
    print("ST3215 Gripper Debug Calibration")
    print("-" * 40)
    
    # Connect
    port = PortHandler(DEVICE)
    pkt = PacketHandler(PROTOCOL_END)
    
    if not port.openPort() or not port.setBaudRate(BAUDRATE):
        print("ERROR: Cannot open port")
        return
    
    # Test connection
    model, res, _ = pkt.ping(port, SERVO_ID)
    if res != COMM_SUCCESS:
        print("ERROR: Servo not responding")
        port.closePort()
        return
    print(f"✓ Connected to servo (Model: {model})")
    
    # Enable torque
    res, _ = pkt.write1ByteTxRx(port, SERVO_ID, TORQUE_ENABLE, 1)
    print(f"✓ Torque enabled (result: {res})")
    
    # Get starting position
    start_pos, _, _ = pkt.read2ByteTxRx(port, SERVO_ID, PRESENT_POS_L)
    print(f"✓ Starting position: {start_pos}")
    
    print("\nSimple movement test:")
    print("Type a number (0-4095) and press Enter to move to that position")
    print("Type 'q' to quit\n")
    
    while True:
        try:
            user_input = input("Position (or 'q' to quit): ").strip()
            if user_input.lower() == 'q':
                break
                
            target = int(user_input)
            if 0 <= target <= 4095:
                print(f"  Moving to {target}...")
                
                # Send command
                res, _ = pkt.write2ByteTxRx(port, SERVO_ID, GOAL_POS_L, target)
                print(f"  Command sent (result: {res})")
                
                # Wait and check
                time.sleep(0.5)
                actual, _, _ = pkt.read2ByteTxRx(port, SERVO_ID, PRESENT_POS_L)
                print(f"  Current position: {actual}")
                print(f"  Difference: {abs(actual - target)}")
            else:
                print("  ERROR: Position must be 0-4095")
                
        except ValueError:
            print("  ERROR: Please enter a number")
        except KeyboardInterrupt:
            break
    
    # Disable torque
    pkt.write1ByteTxRx(port, SERVO_ID, TORQUE_ENABLE, 0)
    port.closePort()
    print("\n✓ Done")

if __name__ == "__main__":
    main() 