#!/usr/bin/env python3
"""
Simple script to read raw Dynamixel positions directly from hardware.
This bypasses all transformations and shows the actual servo positions.
"""

import time
import sys
import argparse
from pathlib import Path

# Ensure the PiperGello repo is on PYTHONPATH so we can import the driver
gello_path = Path(__file__).parent / "PiperGello"
sys.path.insert(0, str(gello_path))

try:
    from gello.dynamixel.driver import DynamixelDriver
except ImportError as e:
    print(f"Error importing Gello driver: {e}")
    print("Make sure the PiperGello repository is available in /app/PiperGello or alongside this script.")
    sys.exit(1)

def parse_args():
    p = argparse.ArgumentParser(description="Read raw Dynamixel joint angles")
    p.add_argument("--port", default="/dev/ttyUSB0", help="Serial device path (default /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=57600, help="Baud rate (default 57600)")
    p.add_argument(
        "--ids",
        default="1,2,3,4,5,6,7",
        help="Comma-separated list of servo IDs to poll (default 1-7)",
    )
    p.add_argument("--rate", type=float, default=10.0, help="Polling rate in Hz (default 10)")
    return p.parse_args()

def main():
    args = parse_args()

    joint_ids = tuple(int(s.strip()) for s in args.ids.split(",") if s.strip())
    
    print(f"Connecting to Dynamixel servos on {args.port}")
    print(f"Joint IDs: {joint_ids}")
    print("Press Ctrl+C to stop\n")
    
    driver = None
    try:
        # Create driver and connect immediately (lazy_connect=False)
        driver = DynamixelDriver(joint_ids, port=args.port, baudrate=args.baud)
        
        # Disable torque for safety (read-only)
        try:
            driver.set_torque_mode(False)
        except Exception:
            pass  # some setups may already have torque off
        
        print("Reading raw Dynamixel positions (radians):")
        print("-" * 70)
        
        period = 1.0 / args.rate if args.rate > 0 else 0.1
        count = 0
        while True:
            raw_positions = driver.get_joints()
            formatted = [f"{pos:7.3f}" for pos in raw_positions]
            print(f"#{count:4d}: [{', '.join(formatted)}]")
            count += 1
            time.sleep(period)
                
    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Failed to connect/read: {e}\n")
        print("Troubleshooting:")
        print(f"  • Does {args.port} exist and have permissions?")
        print("  • Is another process using the Dynamixels?")
        print("  • Are the IDs correct?")
    finally:
        if driver is not None:
            try:
                driver.close()
            except Exception:
                pass

if __name__ == "__main__":
    main()