#!/usr/bin/env python3
"""Test script to debug ST3215 servo connection issues."""
import sys
import time
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

def test_servo_connection(device, baudrate=1000000):
    """Test connection to ST3215 servo on different IDs."""
    print(f"Testing device: {device}")
    print(f"Baudrate: {baudrate}")
    print("-" * 50)
    
    # Initialize port
    port = PortHandler(device)
    if not port.openPort():
        print(f"ERROR: Cannot open port {device}")
        return False
        
    if not port.setBaudRate(baudrate):
        print(f"ERROR: Cannot set baudrate {baudrate}")
        port.closePort()
        return False
    
    # ST3215 uses protocol 0
    pkt = PacketHandler(0)
    
    found_servos = []
    
    # Try common servo IDs
    print("Scanning for servos...")
    for servo_id in range(1, 10):
        model, res, error = pkt.ping(port, servo_id)
        if res == COMM_SUCCESS:
            print(f"✓ Found servo at ID {servo_id} (Model: {model})")
            found_servos.append(servo_id)
        else:
            print(f"✗ No response from ID {servo_id}")
        time.sleep(0.1)  # Small delay between pings
    
    port.closePort()
    
    if found_servos:
        print(f"\nFound {len(found_servos)} servo(s) at ID(s): {found_servos}")
        return True
    else:
        print("\nNo servos found. Please check:")
        print("1. Servo is powered on")
        print("2. Correct serial device")
        print("3. Wiring connections")
        return False

if __name__ == "__main__":
    device = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"
    test_servo_connection(device) 