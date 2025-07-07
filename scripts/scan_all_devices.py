#!/usr/bin/env python3
"""Comprehensive scanner for all servo devices with proper protocol support."""

import sys
import time
import struct
import glob
import os
import binascii
import argparse

try:
    import serial
except ImportError:
    print("Error: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

# Protocol 1.0 constants (for Piper ST3215)
P1_HEADER = [0xFF, 0xFF]
P1_INST_PING = 0x01

# Protocol 2.0 constants (for Dynamixel)
P2_HEADER = [0xFF, 0xFF, 0xFD, 0x00]
P2_INST_PING = 0x01

# --- Servo SDK imports ---
# Piper ST3215 (Protocol 1.0)
try:
    from scservo_sdk import (
        PortHandler as SCSPortHandler,
        PacketHandler as SCSPacketHandler,
        COMM_SUCCESS as SCS_COMM_SUCCESS,
    )
except ImportError:
    SCSPortHandler = SCSPacketHandler = None  # type: ignore
    SCS_COMM_SUCCESS = -1  # sentinel for missing SDK

# Gello Dynamixel (Protocol 2.0)
try:
    from dynamixel_sdk.port_handler import PortHandler as DXLPortHandler
    from dynamixel_sdk.packet_handler import PacketHandler as DXLPacketHandler
    from dynamixel_sdk.robotis_def import COMM_SUCCESS as DXL_COMM_SUCCESS
except ImportError:
    DXLPortHandler = DXLPacketHandler = None  # type: ignore
    DXL_COMM_SUCCESS = -1

def calculate_checksum_p1(packet):
    """Calculate Dynamixel Protocol 1.0 checksum."""
    return (~sum(packet[2:])) & 0xFF

def calculate_crc16(data):
    """Calculate CRC-16 (CCITT) as used by Dynamixel Protocol 2.0."""
    # Robotis uses the standard CCITT polynomial 0x1021 with initial value 0x0000.
    # Python's built-in binascii implements this via crc_hqx.
    return binascii.crc_hqx(bytes(data), 0)

def create_ping_packet_p1(servo_id):
    """Create a Protocol 1.0 ping packet."""
    packet = P1_HEADER + [servo_id, 0x02, P1_INST_PING]
    packet.append(calculate_checksum_p1(packet))
    return bytes(packet)

def create_ping_packet_p2(servo_id):
    """Create a Protocol 2.0 ping packet."""
    # According to Dynamixel Protocol 2.0, the packet structure is:
    # [HEADER] [ID] [LENGTH_L] [LENGTH_H] [INSTRUCTION] [CRC_L] [CRC_H]
    # For a PING with no parameters, length = 3 (instruction + CRC_L + CRC_H)
    packet = P2_HEADER + [servo_id, 0x03, 0x00, P2_INST_PING]
    crc = calculate_crc16(packet)
    packet.extend([crc & 0xFF, (crc >> 8) & 0xFF])
    return bytes(packet)

def scan_device(device_path, baudrate, protocol, max_id=20, verbose=True):
    """Ping a range of IDs on *device_path* using the official SDKs."""

    # Real device node might disappear between glob and scan (e.g. unplugged cable)
    if not os.path.exists(device_path):
        if verbose:
            print(f"  [SKIP] {device_path} does not exist")
        return []

    found_ids = []

    if protocol == 1:  # Piper ST3215 (SCServo)
        if SCSPortHandler is None:
            if verbose:
                print("  [WARN] scservo_sdk not installed – skipping Protocol 1")
            return []

        port = SCSPortHandler(device_path)
        try:
            if not port.openPort():
                raise RuntimeError("openPort failed")
            if not port.setBaudRate(baudrate):
                raise RuntimeError("setBaudRate failed")
        except Exception as e:
            if verbose:
                print(f"  (unable to open {device_path} @ {baudrate} baud: {e})")
            return []

        handler = SCSPacketHandler(0)
        if verbose:
            print(f"  Protocol 1, {baudrate} baud: ", end="", flush=True)

        for servo_id in range(1, max_id + 1):
            _model, result, _ = handler.ping(port, servo_id)
            if result == SCS_COMM_SUCCESS:
                found_ids.append(servo_id)
                if verbose:
                    print(f"ID{servo_id} ", end="", flush=True)

        port.closePort()

    else:  # Protocol 2.0 (Dynamixel / Gello)
        if DXLPortHandler is None:
            if verbose:
                print("  [WARN] dynamixel_sdk not installed – skipping Protocol 2")
            return []

        port = DXLPortHandler(device_path)
        try:
            if not port.openPort():
                raise RuntimeError("openPort failed")
            if not port.setBaudRate(baudrate):
                raise RuntimeError("setBaudRate failed")
        except Exception as e:
            if verbose:
                print(f"  (unable to open {device_path} @ {baudrate} baud: {e})")
            return []

        handler = DXLPacketHandler(2.0)
        if verbose:
            print(f"  Protocol 2, {baudrate} baud: ", end="", flush=True)

        for servo_id in range(1, max_id + 1):
            _model, result, _ = handler.ping(port, servo_id)
            if result == DXL_COMM_SUCCESS:
                found_ids.append(servo_id)
                if verbose:
                    print(f"ID{servo_id} ", end="", flush=True)

        port.closePort()

    if verbose:
        print("(none found)" if not found_ids else "")

    return found_ids

def get_all_devices(verbose=False):
    """Scan the system for likely USB/serial servo devices.

    Returns a mapping of the *display* path (what the user sees) to the actual
    device node (after resolving symlinks).  Any duplicates are removed.
    """

    # Grab all ttyACM* and ttyUSB* devices
    devices = set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))

    # Include anything that udev exposes in /dev/serial/by-id – this makes
    # the human-readable symlink paths visible in the summary.
    devices.update(glob.glob("/dev/serial/by-id/*"))

    # Build a mapping display_path → real_path, avoiding duplicates that point
    # to the same underlying device.
    unique_devices = {}
    for dev in devices:
        try:
            real_dev = os.path.realpath(dev)
            if real_dev not in unique_devices.values():
                unique_devices[dev] = real_dev
        except Exception:
            unique_devices[dev] = dev

    # Log the collected unique serial devices for visibility
    if unique_devices:
        print("Detected serial devices:")
        for display, real in unique_devices.items():
            if display == real:
                print(f"  {display}")
            else:
                print(f"  {display} -> {real}")
    else:
        print("No serial devices detected under /dev")

    return unique_devices

def main():
    parser = argparse.ArgumentParser(description="Comprehensive servo scanner")
    parser.add_argument(
        "--max-id",
        type=int,
        default=20,
        help="Highest servo ID to test (default: 20)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Show detailed per-device scan output",
    )
    args = parser.parse_args()

    if args.verbose:
        print("=== COMPREHENSIVE SERVO SCANNER ===")
        print("Scanning all devices for Piper (Protocol 1.0) and Gello (Protocol 2.0) servos")
        print(f"Testing servo IDs 1-{args.max_id}")
    
    devices = get_all_devices(verbose=args.verbose)
    
    if not devices:
        print("No serial devices found!")
        return
    
    # Configuration for different servo types
    configs = [
        # Piper ST3215 uses Protocol 1.0 at 1MHz
        {"name": "Piper ST3215", "protocol": 1, "baudrates": [1000000]},
        # Gello Dynamixel uses Protocol 2.0 at 57.6k
        {"name": "Gello Dynamixel", "protocol": 2, "baudrates": [57600]},
    ]
    
    results = {}
    
    for dev_name, real_dev in devices.items():
        if args.verbose:
            print(f"\n{dev_name}")
            if dev_name != real_dev:
                print(f"  → {real_dev}")
            print("-" * 60)
        
        dev_results = []
        
        for config in configs:
            for baudrate in config["baudrates"]:
                found = scan_device(real_dev, baudrate, config["protocol"], max_id=args.max_id, verbose=args.verbose)
                if found:
                    dev_results.append({
                        "type": config["name"],
                        "protocol": config["protocol"],
                        "baudrate": baudrate,
                        "ids": found
                    })
        
        if dev_results:
            results[dev_name] = dev_results
    
    # Print summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    
    if not results:
        print("\nNo servos found on any device!")
        print("\nTroubleshooting:")
        print("1. Ensure servos are powered on")
        print("2. Check cable connections")
        print("3. Verify device permissions (may need sudo)")
        print("4. Check if motors are enabled/powered")
    else:
        total_servos = 0
        for dev_name, dev_results in results.items():
            print(f"\n{dev_name}:")
            for result in dev_results:
                print(f"  {result['type']} (Protocol {result['protocol']}, {result['baudrate']} baud):")
                print(f"    Servo IDs: {', '.join(map(str, result['ids']))}")
                total_servos += len(result['ids'])
        
        print(f"\nTotal servos found: {total_servos}")
        
        # Analysis for dual-arm setup
        print("\n" + "="*60)
        print("DUAL-ARM ANALYSIS")
        print("="*60)
        
        piper_count = sum(len(r['ids']) for dev_results in results.values() 
                         for r in dev_results if r['protocol'] == 1)
        gello_count = sum(len(r['ids']) for dev_results in results.values() 
                         for r in dev_results if r['protocol'] == 2)
        
        print(f"Piper grippers found: {piper_count} (expecting 2)")
        print(f"Gello servos found: {gello_count} (expecting 2)")
        
        if piper_count < 2:
            print("\n⚠️  Missing Piper gripper(s)!")
            print("   - Check /dev/ttyACM1 connection")
            print("   - Verify second Piper gripper power")
            print("   - Try different servo IDs (5, 6, etc)")
            
        if gello_count < 2:
            print("\n⚠️  Missing Gello servo(s)!")
            print("   - Check for second FTDI USB device")
            print("   - Verify second Gello power and connection")
            print("   - May need to scan higher servo IDs")

if __name__ == "__main__":
    main()