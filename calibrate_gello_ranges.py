#!/usr/bin/env python3
"""
Calibration helper for Gello device.
Tracks min/max values for each joint as you move them through their full range.
"""

import time
import sys
import argparse
from pathlib import Path
import numpy as np

# Ensure the PiperGello repo is on PYTHONPATH so we can import the driver
gello_path = Path(__file__).parent / "PiperGello"
sys.path.insert(0, str(gello_path))

try:
    from gello.dynamixel.driver import DynamixelDriver
except ImportError as e:
    print(f"Error importing Gello driver: {e}")
    print("Make sure the PiperGello repository is available")
    sys.exit(1)

def parse_args():
    p = argparse.ArgumentParser(description="Calibrate Gello joint ranges")
    p.add_argument("--port", default="/dev/ttyUSB1", help="Serial device path (default /dev/ttyUSB1)")
    p.add_argument("--baud", type=int, default=57600, help="Baud rate (default 57600)")
    p.add_argument(
        "--ids",
        default="1,2,3,4,5,6,7",
        help="Comma-separated list of servo IDs to poll (default 1-7)",
    )
    p.add_argument("--rate", type=float, default=20.0, help="Polling rate in Hz (default 20)")
    return p.parse_args()

def main():
    args = parse_args()

    joint_ids = tuple(int(s.strip()) for s in args.ids.split(",") if s.strip())
    
    print(f"Gello Calibration Tool")
    print(f"======================")
    print(f"Port: {args.port}")
    print(f"Joint IDs: {joint_ids}")
    print(f"\nMove each joint through its FULL range of motion.")
    print(f"The min/max values will be tracked automatically.")
    print(f"Press Ctrl+C when done.\n")
    
    driver = None
    try:
        # Create driver and connect
        driver = DynamixelDriver(joint_ids, port=args.port, baudrate=args.baud)

        # Disable torque for safety (read-only)
        try:
            driver.set_torque_mode(False)
        except Exception:
            pass
        
        # Initialize min/max tracking
        min_values = np.full(len(joint_ids), np.inf)
        max_values = np.full(len(joint_ids), -np.inf)
        
        # Expected output ranges for Piper robot (from URDF/MuJoCo files)
        expected_ranges = {
            1: (-2.618, 2.168),   # joint1
            2: (0.0, 3.14),       # joint2
            3: (-2.967, 0.0),     # joint3
            4: (-1.745, 1.745),   # joint4
            5: (-1.22, 1.22),     # joint5
            6: (-2.0944, 2.0944), # joint6
            7: (0.0, 1.0),        # gripper (normalized)
        }
        
        period = 1.0 / args.rate if args.rate > 0 else 0.05
        count = 0
        
        print("Current ranges (move joints to update):")
        print("-" * 80)
        
        while True:
            raw_positions = driver.get_joints()
            
            # Update min/max
            for i, pos in enumerate(raw_positions):
                min_values[i] = min(min_values[i], pos)
                max_values[i] = max(max_values[i], pos)
            
            # Clear screen and show updated info
            print("\033[H\033[J", end='')  # Clear screen
            print(f"Gello Calibration - Move each joint through FULL range - Ctrl+C when done\n")
            
            print(f"{'Joint':<8} {'Current':>8} {'Min':>8} {'Max':>8} {'Range':>8}")
            print("-" * 48)
            
            for i, joint_id in enumerate(joint_ids[:6]):  # Only show arm joints
                current = raw_positions[i]
                min_val = min_values[i] if min_values[i] != np.inf else current
                max_val = max_values[i] if max_values[i] != -np.inf else current
                range_val = max_val - min_val
                
                # Highlight joints that haven't moved much
                marker = "✓" if range_val > 0.5 else ("~" if range_val > 0.1 else "✗")
                
                print(f"Joint {joint_id}  {marker}  {current:8.3f} {min_val:8.3f} {max_val:8.3f} {range_val:8.3f}")
            
            # Only show calibration data if we have enough movement
            calibrated_count = sum(1 for i in range(6) if max_values[i] - min_values[i] > 0.5)
            
            if calibrated_count >= 4:  # Show config if at least 4 joints calibrated
                print("\n" + "-" * 48)
                print("Calculated offsets:")
                offsets = []
                signs = []
                for i in range(6):
                    if joint_ids[i] in expected_ranges and max_values[i] - min_values[i] > 0.1:
                        raw_min = min_values[i]
                        raw_max = max_values[i]
                        exp_min, exp_max = expected_ranges[joint_ids[i]]
                        
                        # Calculate offset and sign
                        raw_increasing = raw_max > raw_min
                        exp_increasing = exp_max > exp_min
                        sign = 1 if raw_increasing == exp_increasing else -1
                        
                        if sign == 1:
                            offset = raw_min - exp_min
                        else:
                            offset = raw_max - exp_min
                        
                        offsets.append(f"{offset:.3f}")
                        signs.append(str(sign))
                    else:
                        offsets.append("0.0")
                        signs.append("1")
                
                print(f"joint_offsets = ({', '.join(offsets)})")
                print(f"joint_signs = ({', '.join(signs)})")
            else:
                print(f"\nCalibrated: {calibrated_count}/6 joints (need more movement)")
            
            print(f"\n✓=good range  ~=some movement  ✗=needs movement")
            
            count += 1
            time.sleep(period)
                
    except KeyboardInterrupt:
        print("\n\nCalibration complete!")
        print("\nFinal ranges:")
        print("-" * 60)
        for i, joint_id in enumerate(joint_ids):
            if max_values[i] - min_values[i] > 0.1:
                print(f"Joint {joint_id}: [{min_values[i]:.3f}, {max_values[i]:.3f}] (range: {max_values[i]-min_values[i]:.3f})")
            else:
                print(f"Joint {joint_id}: NOT CALIBRATED (insufficient movement)")

        # Offer to capture zero offsets
        try:
            capture = input("\nCapture joint offsets now (move each joint to zero)? [y/N]: ").strip().lower()
        except EOFError:
            capture = 'n'

        if capture == 'y':
            print("\nOffset capture: move each joint (1-6) to its mechanical zero, then press <Enter>.\n")
            offsets = []
            for idx, jid in enumerate(joint_ids[:6]):
                input(f"Joint {jid}: press <Enter> when at zero …")
                raw = driver.get_joints()[idx]
                print(f"  Captured offset for joint {jid}: {raw:.3f} rad")
                offsets.append(raw)

            print("\nCopy this block into your YAML 'gello' section:")
            print("joint_offsets: [" + ", ".join(f"{v:.3f}" for v in offsets) + "]")
            print("joint_signs:   [1, 1, 1, 1, 1, 1]  # update signs after testing")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if driver is not None:
            try:
                driver.close()
            except Exception:
                pass

if __name__ == "__main__":
    main() 