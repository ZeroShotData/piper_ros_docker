#!/usr/bin/env python3
"""
Unified Gripper Configuration and Calibration Tool
==================================================

This tool calibrates both Piper (ST3215) and Gello (Dynamixel) grippers
and saves the configuration to a YAML file for use with the launch system.

Usage:
------
# Full calibration (both grippers)
python3 create_gripper_config.py --config-name my_gripper_setup

# Calibrate only Piper
python3 create_gripper_config.py --config-name my_setup --piper-only

# Calibrate only Gello  
python3 create_gripper_config.py --config-name my_setup --gello-only

# Load existing config and modify
python3 create_gripper_config.py --load-config configs/existing.yaml
"""

import argparse
import yaml
import os, sys, glob, errno
import termios
import tty
import time
import math
from datetime import datetime
from pathlib import Path
import numpy as np

# For Piper ST3215 servo
try:
    from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    PIPER_AVAILABLE = True
except ImportError:
    PIPER_AVAILABLE = False

# For Gello Dynamixel servo - only import when needed
GELLO_AVAILABLE = False
DynamixelDriver = None


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


def check_gello_available():
    """Check if Gello/Dynamixel driver is available"""
    global GELLO_AVAILABLE, DynamixelDriver
    if not GELLO_AVAILABLE:
        try:
            from gello.dynamixel.driver import DynamixelDriver as DynDriver
            DynamixelDriver = DynDriver
            GELLO_AVAILABLE = True
        except ImportError:
            GELLO_AVAILABLE = False
    return GELLO_AVAILABLE


def main():
    parser = argparse.ArgumentParser(
        description='Unified gripper calibration and configuration tool'
    )
    
    # Config file options
    config_group = parser.add_mutually_exclusive_group(required=True)
    config_group.add_argument('--config-name', type=str,
                            help='Name for new config (will be saved to configs/)')
    config_group.add_argument('--load-config', type=str,
                            help='Load and modify existing config file')
    
    # Calibration options
    parser.add_argument('--piper-only', action='store_true',
                       help='Only calibrate Piper gripper')
    parser.add_argument('--gello-only', action='store_true', 
                       help='Only calibrate Gello gripper')
    
    # Piper options
    parser.add_argument('--piper-device', default='/dev/ttyACM0',
                       help='Piper servo device')
    parser.add_argument('--piper-id', type=int, default=6,
                       help='Piper servo ID')
    parser.add_argument('--piper-torque', type=int, default=1000,
                       help='Default Piper torque limit')
    
    # Gello options  
    parser.add_argument('--gello-port', 
                       default='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NMKV-if00-port0',
                       help='Gello serial port (symlink will be auto-created if missing)')
    parser.add_argument('--gello-id', type=int, default=7,
                       help='Gello servo ID')
    parser.add_argument('--gello-effort-min', type=float, default=0.5,
                       help='Minimum effort clamp value')
    parser.add_argument('--gello-effort-max', type=float, default=3.0,
                       help='Maximum effort clamp value')
    
    # Metadata
    parser.add_argument('--description', type=str, default='',
                       help='Description of this gripper configuration')
    parser.add_argument('--robot-serial', type=str, default='PIPER_001',
                       help='Robot serial number')
    
    args = parser.parse_args()
    
    # Check dependencies based on what we need to calibrate
    if not args.gello_only and not PIPER_AVAILABLE:
        print("ERROR: scservo_sdk not installed (required for Piper calibration)")
        print("Please run: pip install feetech-servo-sdk")
        print("Or run with --gello-only to skip Piper calibration")
        sys.exit(1)
    
    if not args.piper_only and not check_gello_available():
        print("WARNING: gello.dynamixel.driver not found")
        print("Gello calibration will be skipped")
        print("To enable Gello calibration:")
        print("  - Ensure PiperGello is installed")
        print("  - Add it to PYTHONPATH if needed")
        print("  - Or run with --piper-only to skip this warning")
        if not args.gello_only:
            if input("\nContinue with Piper calibration only? (y/N): ").lower() != 'y':
                sys.exit(1)
        else:
            print("ERROR: Cannot calibrate Gello without gello.dynamixel.driver")
            sys.exit(1)
    
    # Load or create config
    if args.load_config:
        with open(args.load_config, 'r') as f:
            config = yaml.safe_load(f)
        config_path = args.load_config
    else:
        config = {
            'metadata': {
                'config_name': args.config_name,
                'created_date': datetime.now().isoformat(),
                'robot_serial': args.robot_serial,
                'description': args.description
            }
        }
        # Ensure configs directory exists
        os.makedirs('configs', exist_ok=True)
        config_path = f'configs/{args.config_name}.yaml'
    
    # Calibrate Piper
    if not args.gello_only and PIPER_AVAILABLE:
        print("\n" + "="*60)
        print("PIPER GRIPPER CALIBRATION (ST3215)")
        print("="*60)
        
        try:
            piper_open, piper_close = calibrate_piper_gripper(
                device=args.piper_device,
                servo_id=args.piper_id
            )
            
            config['piper_gripper'] = {
                'servo_id': args.piper_id,
                'device': args.piper_device,
                'open_ticks': piper_open,
                'close_ticks': piper_close,
                'default_torque': args.piper_torque
            }
            
            print(f"\n✅ Piper calibration complete:")
            print(f"   Open: {piper_open} ticks")
            print(f"   Close: {piper_close} ticks")
        except Exception as e:
            print(f"\n❌ Piper calibration failed: {e}")
            if not args.piper_only and GELLO_AVAILABLE:
                if input("Continue with Gello calibration? (y/N): ").lower() != 'y':
                    return
    
    # Calibrate Gello
    if not args.piper_only and GELLO_AVAILABLE:
        print("\n" + "="*60)
        print("GELLO GRIPPER CALIBRATION (Dynamixel)")
        print("="*60)
        
        try:
            # Ensure by-id symlink exists if missing
            args.gello_port = ensure_gello_symlink(args.gello_port)
            gello_open, gello_close = calibrate_gello_gripper(
                port=args.gello_port,
                servo_id=args.gello_id
            )
            
            config['gello_gripper'] = {
                'servo_id': args.gello_id,
                'port': args.gello_port,
                'open_degrees': gello_open,
                'close_degrees': gello_close,
                'effort_range': [args.gello_effort_min, args.gello_effort_max]
            }
            
            print(f"\n✅ Gello calibration complete:")
            print(f"   Open: {gello_open:.1f}°")
            print(f"   Close: {gello_close:.1f}°")
        except Exception as e:
            print(f"\n❌ Gello calibration failed: {e}")
    
    # -----------------------------------------------------------------
    # Confirm save
    # -----------------------------------------------------------------
    if input(f"\nSave this configuration to '{config_path}'? (y/N): ").lower() != 'y':
        print("\n⚠️  Configuration NOT saved – exiting.")
        return

    with open(config_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    print("\n" + "="*60)
    print("CALIBRATION COMPLETE")
    print("="*60)
    print(f"📁 Configuration saved to: {config_path}")
    print("\n📋 To use this configuration:")
    print(f"   ros2 launch piper start_piper.launch.py \\")
    print(f"       gripper_exist:=true \\")
    print(f"       gripper_config:={os.path.abspath(config_path)} \\")
    print(f"       gello_exist:=true")
    print("="*60)
    
    # Optional: Test the configuration
    if input("\nTest the configuration? (y/N): ").lower() == 'y':
        test_configuration(config)


def calibrate_piper_gripper(device, servo_id):
    """
    Interactive Piper calibration - returns (open_ticks, close_ticks)
    """
    # ST3215 servo constants
    BAUDRATE = 1_000_000
    PROTOCOL_END = 0
    GOAL_POS_L = 42
    PRESENT_POS_L = 56
    TORQUE_ENABLE = 40
    
    # Default positions
    DEFAULT_OPEN = 1592
    DEFAULT_CLOSE = 900
    
    print(f"\n🔌 Connecting to ST3215 servo...")
    print(f"   Device: {device}")
    print(f"   ID: {servo_id}")
    
    port_handler = PortHandler(device)
    packet_handler = PacketHandler(PROTOCOL_END)
    
    if not port_handler.openPort():
        raise RuntimeError(f"Failed to open port {device}")
    
    if not port_handler.setBaudRate(BAUDRATE):
        port_handler.closePort()
        raise RuntimeError(f"Failed to set baud rate {BAUDRATE}")
    
    # Test connection
    time.sleep(0.1)
    model, res, _ = packet_handler.ping(port_handler, servo_id)
    if res != COMM_SUCCESS:
        port_handler.closePort()
        raise RuntimeError(f"Servo ID {servo_id} not responding on {device}")
    
    print(f"✅ Connected to servo (Model: {model})")
    
    # Enable torque
    packet_handler.write1ByteTxRx(port_handler, servo_id, TORQUE_ENABLE, 1)
    
    # Get current position
    current_pos, _, _ = packet_handler.read2ByteTxRx(port_handler, servo_id, PRESENT_POS_L)
    
    # Initialize calibration values
    test_open = DEFAULT_OPEN
    test_close = DEFAULT_CLOSE
    last_toggle = test_open
    
    print(f"\n📍 Starting position: {current_pos} ticks")
    print("\n" + "="*40)
    print("PIPER CALIBRATION INSTRUCTIONS:")
    print("="*40)
    print("← / a : Move -10 ticks (towards open)")
    print("→ / d : Move +10 ticks (towards close)")
    print("↓ / s : Move -50 ticks (big step open)")
    print("↑ / w : Move +50 ticks (big step close)")
    print("o     : Jump to current OPEN position")
    print("c     : Jump to current CLOSE position")
    print("space : Toggle between open/close")
    print("[     : Set current as OPEN position")
    print("]     : Set current as CLOSE position")
    print("q     : Save and continue")
    print("="*40)
    
    try:
        with KeyReader() as reader:
            while True:
                key = reader.read_key()
                
                if key == 'q':
                    break
                
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
                else:
                    continue
                
                # Apply movement
                if delta != 0:
                    new_pos = current_pos + delta
                
                # Enforce limits
                new_pos = max(0, min(4095, new_pos))
                
                # Send command to servo
                result, _ = packet_handler.write2ByteTxRx(port_handler, servo_id, GOAL_POS_L, new_pos)
                if result == COMM_SUCCESS:
                    current_pos = new_pos
                    if delta != 0:
                        print(f"Position: {current_pos:4d} (Open:{test_open:4d} Close:{test_close:4d})", end='\r')
                
                time.sleep(0.05)
                
    finally:
        # Disable torque before closing
        packet_handler.write1ByteTxRx(port_handler, servo_id, TORQUE_ENABLE, 0)
        port_handler.closePort()
    
    return test_open, test_close


def calibrate_gello_gripper(port, servo_id):
    """
    Interactive Gello calibration - returns (open_degrees, close_degrees)
    """
    if not GELLO_AVAILABLE:
        raise RuntimeError("Gello calibration requires gello.dynamixel.driver")
    
    print(f"\n🔌 Connecting to Dynamixel servo...")
    print(f"   Port: {port}")
    print(f"   ID: {servo_id}")
    
    driver = DynamixelDriver([servo_id], port=port, baudrate=57600)
    driver.set_torque_mode(True)
    
    # Get current position
    curr_angle = driver.get_joints()[0]
    open_angle = curr_angle
    close_angle = curr_angle
    last_toggle = open_angle
    
    def deg(rad):
        return math.degrees(rad)
    
    print(f"\n📍 Starting position: {deg(curr_angle):.1f}°")
    print("\n" + "="*40)
    print("GELLO CALIBRATION INSTRUCTIONS:")
    print("="*40)
    print("← / a : Rotate -2° (towards open)")
    print("→ / d : Rotate +2° (towards close)")
    print("↓ / s : Rotate -10° (big step open)")
    print("↑ / w : Rotate +10° (big step close)")
    print("o     : Jump to current OPEN position")
    print("c     : Jump to current CLOSE position")
    print("space : Toggle between open/close")
    print("[     : Set current as OPEN position")
    print("]     : Set current as CLOSE position")
    print("q     : Save and continue")
    print("="*40)
    
    try:
        with KeyReader() as reader:
            while True:
                key = reader.read_key()
                
                if key == 'q':
                    break
                
                step_deg = 2
                big_step_deg = 10
                moved = False
                target = curr_angle
                
                if key in ('a',):  # smaller (open)
                    target = curr_angle - math.radians(step_deg)
                    moved = True
                elif key in ('d',):  # larger (close)
                    target = curr_angle + math.radians(step_deg)
                    moved = True
                elif key in ('s',):  # big open
                    target = curr_angle - math.radians(big_step_deg)
                    moved = True
                elif key in ('w',):  # big close
                    target = curr_angle + math.radians(big_step_deg)
                    moved = True
                elif key == 'o':
                    target = open_angle
                    moved = True
                    print(f"\n➡️  Testing OPEN position: {deg(open_angle):.1f}°")
                elif key == 'c':
                    target = close_angle
                    moved = True
                    print(f"\n➡️  Testing CLOSE position: {deg(close_angle):.1f}°")
                elif key == ' ':
                    if abs(deg(open_angle - close_angle)) < 1e-3:
                        print("\n⚠️  Set open [ and close ] positions first – toggle skipped")
                        continue
                    target = close_angle if last_toggle == open_angle else open_angle
                    last_toggle = target
                    moved = True
                    print(f"\n🔄 Toggle to {'CLOSE' if target == close_angle else 'OPEN'}: {deg(target):.1f}°")
                elif key == '[':
                    open_angle = curr_angle
                    print(f"\n✅ Set OPEN position: {deg(open_angle):.1f}°")
                elif key == ']':
                    close_angle = curr_angle
                    print(f"\n✅ Set CLOSE position: {deg(close_angle):.1f}°")
                else:
                    continue
                
                if moved:
                    driver.set_single_joint_position(servo_id, target)
                    curr_angle = target
                    print(f"Angle: {deg(curr_angle):6.1f}° (Open:{deg(open_angle):6.1f}° Close:{deg(close_angle):6.1f}°)", end='\r')
                
                time.sleep(0.05)
                
    finally:
        driver.set_torque_mode(False)
        driver.close()
    
    return deg(open_angle), deg(close_angle)


def test_configuration(config):
    """Test the calibrated values by moving grippers"""
    print("\n🧪 Testing configuration...")
    print("="*40)
    
    # Test Piper if configured
    if 'piper_gripper' in config and PIPER_AVAILABLE:
        print("\nTesting Piper gripper...")
        piper_cfg = config['piper_gripper']
        
        try:
            port_handler = PortHandler(piper_cfg['device'])
            packet_handler = PacketHandler(0)
            
            if port_handler.openPort() and port_handler.setBaudRate(1_000_000):
                # Enable torque
                packet_handler.write1ByteTxRx(port_handler, piper_cfg['servo_id'], 40, 1)
                
                # Test open position
                print(f"  Moving to OPEN ({piper_cfg['open_ticks']} ticks)...")
                packet_handler.write2ByteTxRx(port_handler, piper_cfg['servo_id'], 42, piper_cfg['open_ticks'])
                time.sleep(1.5)
                
                # Test close position
                print(f"  Moving to CLOSE ({piper_cfg['close_ticks']} ticks)...")
                packet_handler.write2ByteTxRx(port_handler, piper_cfg['servo_id'], 42, piper_cfg['close_ticks'])
                time.sleep(1.5)
                
                # Return to open
                print("  Returning to OPEN...")
                packet_handler.write2ByteTxRx(port_handler, piper_cfg['servo_id'], 42, piper_cfg['open_ticks'])
                time.sleep(1.0)
                
                # Disable torque
                packet_handler.write1ByteTxRx(port_handler, piper_cfg['servo_id'], 40, 0)
                port_handler.closePort()
                
                print("  ✅ Piper test complete")
                
        except Exception as e:
            print(f"  ❌ Piper test failed: {e}")
    
    # Test Gello if configured
    if 'gello_gripper' in config and GELLO_AVAILABLE:
        print("\nTesting Gello gripper...")
        gello_cfg = config['gello_gripper']
        
        try:
            driver = DynamixelDriver([gello_cfg['servo_id']], port=gello_cfg['port'])
            driver.set_torque_mode(True)
            
            # Test open position
            print(f"  Moving to OPEN ({gello_cfg['open_degrees']:.1f}°)...")
            driver.set_single_joint_position(gello_cfg['servo_id'], math.radians(gello_cfg['open_degrees']))
            time.sleep(1.5)
            
            # Test close position
            print(f"  Moving to CLOSE ({gello_cfg['close_degrees']:.1f}°)...")
            driver.set_single_joint_position(gello_cfg['servo_id'], math.radians(gello_cfg['close_degrees']))
            time.sleep(1.5)
            
            # Return to open
            print("  Returning to OPEN...")
            driver.set_single_joint_position(gello_cfg['servo_id'], math.radians(gello_cfg['open_degrees']))
            time.sleep(1.0)
            
            driver.set_torque_mode(False)
            driver.close()
            
            print("  ✅ Gello test complete")
            
        except Exception as e:
            print(f"  ❌ Gello test failed: {e}")
    
    print("\n✅ Testing complete!")


# ------------------------------------------------------------
# Helpers – USB serial symlink
# ------------------------------------------------------------


def ensure_gello_symlink(port_path: str) -> str:
    """If *port_path* is the default /dev/serial/by-id path but the file is
    missing, try to create a symlink that points to a /dev/ttyUSB* device.

    Returns the (possibly updated) path that should now exist.
    """

    if os.path.exists(port_path):
        return port_path  # nothing to do

    by_id_prefix = "/dev/serial/by-id/"
    if not port_path.startswith(by_id_prefix):
        return port_path  # raw device or custom path; leave unchanged

    # Attempt to resolve to first ttyUSB* device
    candidates = sorted(glob.glob("/dev/ttyUSB*")) or sorted(glob.glob("/dev/ttyACM*"))
    if not candidates:
        return port_path  # nothing we can link to

    target = candidates[0]
    try:
        os.makedirs(by_id_prefix, exist_ok=True)
        os.symlink(target, port_path, target_is_directory=False)
        print(f"[INFO] Created symlink: {port_path} -> {target}")
    except FileExistsError:
        pass  # created in parallel
    except PermissionError as e:
        print(f"[WARN] Cannot create symlink {port_path}: {e}")

    return port_path


if __name__ == "__main__":
    main()