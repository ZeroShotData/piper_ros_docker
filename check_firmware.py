#!/usr/bin/env python3
"""Check firmware versions of both Piper arms"""

import sys
import time
import subprocess

# Install piper_sdk if not already installed
try:
    from piper_sdk import C_PiperInterface
except ImportError:
    print("Installing piper_sdk...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "piper_sdk"])
    from piper_sdk import C_PiperInterface

def check_arm_firmware(can_port, arm_name):
    """Check firmware version for a specific arm"""
    print(f"\n{'='*60}")
    print(f"Checking {arm_name} arm on {can_port}")
    print('='*60)
    
    try:
        # Create interface
        piper = C_PiperInterface(can_name=can_port, judge_flag=False)
        
        # Connect to the arm
        print(f"Connecting to {can_port}...")
        piper.ConnectPort(can_init=False, piper_init=True, start_thread=True)
        
        # Wait for initialization
        time.sleep(2)
        
        # Query firmware version
        print("Querying firmware version...")
        piper.SearchPiperFirmwareVersion()
        time.sleep(1)
        
        # Get firmware version
        version = piper.GetPiperFirmwareVersion()
        
        if version == -0x4AF:
            print(f"❌ Failed to get firmware version (timeout or no response)")
        else:
            print(f"✅ Firmware version: {version}")
            
            # Determine which interface version should be used
            if "V1.5-2" in str(version) or "V1.5.2" in str(version):
                print("   → Should use C_PiperInterface_V2 (newer firmware)")
            else:
                print("   → Should use C_PiperInterface (older firmware)")
        
        # Check arm status
        status = piper.GetArmStatus()
        if status:
            print(f"   Arm status code: {status.arm_status.arm_status}")
            print(f"   Control mode: {status.arm_status.ctrl_mode}")
        
        # Disconnect
        piper.DisconnectPort()
        
    except Exception as e:
        print(f"❌ Error checking {arm_name} arm: {e}")

def main():
    print("Piper Arm Firmware Version Checker")
    print("==================================")
    
    # Check right arm (baguette)
    check_arm_firmware("can_right", "RIGHT (baguette)")
    
    # Check left arm (choupette) 
    check_arm_firmware("can_left", "LEFT (choupette)")
    
    print("\n" + "="*60)
    print("DIAGNOSIS")
    print("="*60)
    print("""
If the firmware versions are different:
- The ROS node uses C_PiperInterface (for firmware <= V1.5-1)
- Newer firmware (>= V1.5-2) may need C_PiperInterface_V2
- This mismatch could cause the arm to not initialize properly

Solutions:
1. Update both arms to the same firmware version
2. Modify the ROS node to use the correct interface version
3. Use a compatibility mode if available
""")

if __name__ == "__main__":
    main() 