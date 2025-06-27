#!/usr/bin/env python3
"""Test basic servo movement and torque control."""
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
MOVING_STATUS = 122  # Check if servo is moving

def test_servo():
    print(f"Testing servo {SERVO_ID} on {DEVICE}")
    
    # Connect
    port = PortHandler(DEVICE)
    pkt = PacketHandler(PROTOCOL_END)
    
    if not port.openPort():
        print("ERROR: Cannot open port")
        return
        
    if not port.setBaudRate(BAUDRATE):
        print("ERROR: Cannot set baudrate")
        port.closePort()
        return
    
    # Test ping
    model, res, _ = pkt.ping(port, SERVO_ID)
    if res != COMM_SUCCESS:
        print("ERROR: Servo not responding")
        port.closePort()
        return
    print(f"✓ Servo found (Model: {model})")
    
    # Enable torque
    res, _ = pkt.write1ByteTxRx(port, SERVO_ID, TORQUE_ENABLE, 1)
    if res != COMM_SUCCESS:
        print("ERROR: Cannot enable torque")
        port.closePort()
        return
    print("✓ Torque enabled")
    
    # Read torque status
    torque_status, res, _ = pkt.read1ByteTxRx(port, SERVO_ID, TORQUE_ENABLE)
    print(f"  Torque status register: {torque_status}")
    
    # Get current position
    current_pos, res, _ = pkt.read2ByteTxRx(port, SERVO_ID, PRESENT_POS_L)
    print(f"✓ Current position: {current_pos}")
    
    # Test movement
    print("\nTesting movement...")
    test_positions = [1000, 2000, 3000, 2000, current_pos]
    
    for i, target_pos in enumerate(test_positions):
        print(f"\nMove {i+1}: Target = {target_pos}")
        
        # Send command
        res, _ = pkt.write2ByteTxRx(port, SERVO_ID, GOAL_POS_L, target_pos)
        if res != COMM_SUCCESS:
            print("  ERROR: Failed to send command")
            continue
        
        # Wait and check position
        time.sleep(1.0)  # Give servo time to move
        
        # Read actual position
        actual_pos, res, _ = pkt.read2ByteTxRx(port, SERVO_ID, PRESENT_POS_L)
        if res == COMM_SUCCESS:
            print(f"  Actual position: {actual_pos}")
            print(f"  Moved: {'YES' if abs(actual_pos - current_pos) > 10 else 'NO'}")
            current_pos = actual_pos
        
        # Check moving status
        moving, res, _ = pkt.read1ByteTxRx(port, SERVO_ID, MOVING_STATUS)
        if res == COMM_SUCCESS:
            print(f"  Moving status: {moving}")
    
    # Disable torque
    pkt.write1ByteTxRx(port, SERVO_ID, TORQUE_ENABLE, 0)
    port.closePort()
    print("\n✓ Test complete")

if __name__ == "__main__":
    test_servo() 