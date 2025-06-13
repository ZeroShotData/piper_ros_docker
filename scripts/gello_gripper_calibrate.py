#!/usr/bin/env python3
"""
GELLO Dynamixel Gripper Calibration Tool
=======================================

This interactive script lets you find and record the **open (top)** and
**closed (down)** angles for the Dynamixel-based GELLO gripper.
It is modelled after `gripper_calibrate.py` (ST3215) but uses the
Dynamixel protocol via the in-tree `gello.dynamixel.driver` module.

Usage (inside Docker or host with serial permissions):
------------------------------------------------------
  python3 gello_gripper_calibrate.py                 # defaults
  python3 gello_gripper_calibrate.py --port /dev/ttyUSB0 --id 7

Keys during calibration:
  ← / a : rotate –2° (towards OPEN)
  → / d : rotate +2° (towards CLOSE)
  ↓ / s : rotate –10°
  ↑ / w : rotate +10°
  o     : jump to current OPEN candidate
  c     : jump to current CLOSE candidate
  space : toggle between OPEN/CLOSE candidates
  [     : save current position as OPEN (top)
  ]     : save current position as CLOSE (down)
  q     : quit and print results
  h/?   : help

After calibration copy the reported angles (degrees) into
`gello/agents/gello_agent.py` (gripper_config for your port).

Author: Piper Robotics – Adapted for GELLO
License: MIT
"""

import argparse
import sys
import termios
import tty
import time
import math
from pathlib import Path

import numpy as np

# Use the high-level driver that is already in this repo
from gello.dynamixel.driver import DynamixelDriver

# ---------------- Utilities ----------------

class KeyReader:
    """Context manager for reading single keystrokes (non-blocking)."""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self):
        ch = sys.stdin.read(1)
        # Arrow keys start with ESC [ A/B/C/D
        if ch == "\x1b":
            ch2 = sys.stdin.read(1)
            if ch2 == "[":
                ch3 = sys.stdin.read(1)
                return {"A": "w", "B": "s", "C": "d", "D": "a"}.get(ch3, "")
        return ch.lower()

# ---------------- Main logic ----------------

def move_and_wait(driver: DynamixelDriver, sid: int, angle_rad: float, wait: float = 0.05):
    """Send single-servo position command and sleep a little so the movement happens."""
    try:
        driver.set_single_joint_position(sid, angle_rad)
        time.sleep(wait)
    except Exception as e:
        print(f"❌  Failed to move servo {sid}: {e}")


def main():
    parser = argparse.ArgumentParser(description="Interactive calibration for GELLO Dynamixel gripper")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port of the Dynamixel U2D2/FTDI")
    parser.add_argument("--baudrate", type=int, default=57600, help="Baud rate (default 57600)")
    parser.add_argument("--id", type=int, default=7, help="Servo ID of the gripper (default 7)")
    args = parser.parse_args()

    print("Connecting to Dynamixel…")
    try:
        driver = DynamixelDriver([args.id], port=args.port, baudrate=args.baudrate)
        driver.set_torque_mode(True)
    except Exception as e:
        print(f"❌  Unable to connect/enable torque: {e}")
        sys.exit(1)

    # Initial position (current)
    curr_angle = driver.get_joints()[0]
    open_angle = curr_angle  # candidate open
    close_angle = curr_angle  # candidate close
    last_toggle = open_angle

    def deg(rad):
        return math.degrees(rad)

    HELP_SHORT = """\nControls: a/d=±2°, w/s=±10°, [ set OPEN, ] set CLOSE, space toggle, q quit, h help\n"""
    print(HELP_SHORT)

    with KeyReader() as kr:
        while True:
            key = kr.read_key()
            if key == "q":
                break
            if key in ("h", "?"):
                print(__doc__)
                continue

            step_deg = 2  # fine step
            big_step_deg = 10  # coarse
            moved = False
            target = curr_angle

            if key in ("a",):  # smaller (open)
                target = curr_angle - math.radians(step_deg)
                moved = True
            elif key in ("d",):  # larger (close)
                target = curr_angle + math.radians(step_deg)
                moved = True
            elif key in ("s",):  # big open
                target = curr_angle - math.radians(big_step_deg)
                moved = True
            elif key in ("w",):  # big close
                target = curr_angle + math.radians(big_step_deg)
                moved = True
            elif key == "o":
                target = open_angle
                moved = True
            elif key == "c":
                target = close_angle
                moved = True
            elif key == " ":
                target = close_angle if last_toggle == open_angle else open_angle
                last_toggle = target
                moved = True
            elif key == "[":
                open_angle = curr_angle
                print(f"✅  Set OPEN (top) angle: {deg(open_angle):.2f}°")
                continue
            elif key == "]":
                close_angle = curr_angle
                print(f"✅  Set CLOSE (down) angle: {deg(close_angle):.2f}°")
                continue
            else:
                continue

            if moved:
                # Send command and update curr_angle
                move_and_wait(driver, args.id, target)
                curr_angle = target
                print(f"Angle: {deg(curr_angle):6.2f}°  | OPEN {deg(open_angle):6.2f}°  CLOSE {deg(close_angle):6.2f}°", end="\r", flush=True)

    print("\n\nCalibration finished.")
    print("Suggested gripper_config entry (degrees):")
    print(f"    gripper_config = ({args.id}, {deg(open_angle):.1f}, {deg(close_angle):.1f})")
    print("Copy this tuple into the relevant PORT_CONFIG_MAP entry in gello/agents/gello_agent.py and restart.")

    # Clean up
    try:
        driver.set_torque_mode(False)
    except Exception:
        pass


if __name__ == "__main__":
    main() 