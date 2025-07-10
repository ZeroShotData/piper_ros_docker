#!/usr/bin/env python3
"""
Recalibrate Joint 2 Zero Position
=================================

This script sets the current position of joint 2 as its new zero position.
Use this when the software zero doesn't match the physical zero position.

Usage:
1. Manually move joint 2 to its true physical zero position
2. Run this script
3. The current position will be saved as the new zero
"""

import time
import sys
from piper_sdk import C_PiperInterface

def main():
    print("Joint 2 Zero Position Recalibration Tool")
    print("=" * 40)
    
    # Check if user is ready
    print("\nBefore proceeding:")
    print("1. Ensure the LEFT arm (choupette) is powered on")
    print("2. Manually move joint 2 to its TRUE ZERO position")
    print("   (where it should be when commanded to 0 degrees)")
    print("3. Hold it steady at that position")
    
    response = input("\nReady to set current position as zero? [y/N]: ").strip().lower()
    if response != 'y':
        print("Calibration cancelled.")
        return 1
    
    try:
        # Connect to the left arm
        print("\nConnecting to left arm (can_left)...")
        piper = C_PiperInterface("can_left")
        piper.ConnectPort()
        
        # Wait for connection
        time.sleep(0.5)
        
        # Enable the arm if needed
        print("Enabling arm...")
        piper.EnableArm(7)  # Enable all motors
        time.sleep(0.5)
        
        # Set joint 2's current position as zero
        print("Setting joint 2 current position as zero...")
        piper.JointConfig(
            joint_num=2,        # Joint 2
            set_zero=0xAE,      # Set current position as zero
            acc_param_is_effective=0,  # Don't change acceleration
            max_joint_acc=500,  # Default value
            clear_err=0         # Don't clear errors
        )
        
        print("✅ Joint 2 zero position has been recalibrated!")
        print("\nThe current physical position is now registered as 0 degrees.")
        print("The joint should now respond correctly to position commands.")
        
        # Disconnect
        piper.DisconnectPort()
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("\nTroubleshooting:")
        print("- Ensure the left arm is powered on")
        print("- Check that can_left is properly configured")
        print("- Try running with sudo if there are permission issues")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 