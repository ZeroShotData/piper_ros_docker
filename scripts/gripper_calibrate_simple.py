#!/usr/bin/env python3
"""Simple ST3215 gripper calibration helper.

Why this script?
----------------
The original interactive tool relies on low-level terminal tricks that do not
work reliably when you run the code inside a Docker container (or over SSH).
This stripped-down helper only uses `input()` so it **always** works – you just
press Enter after each command.

Usage inside the container:
$ docker exec -it piper python3 /app/scripts/gripper_calibrate_simple.py

Workflow:
1. Type a target tick value (0-4095) and hit Enter → the gripper moves there.
2. When you are happy with an OPEN position type `[` and Enter.
3. When you are happy with a CLOSE position type `]` and Enter.
4. Use `q` and Enter to quit – the script prints the two values.

Tip: Typical ST3215 range is roughly 900 (closed) … 1600 (open).
"""
from __future__ import annotations
import sys, time, argparse

try:
    from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    sys.exit("scservo_sdk missing – run inside the Piper container")

DEVICE_DEFAULT = "/dev/ttyACM0"
SERVO_ID_DEFAULT = 6
BAUD = 1_000_000
PROTO = 0

GOAL_POS_L = 42
PRESENT_POS_L = 56
TORQUE_ENABLE = 40

OPEN_INIT = 1592
CLOSE_INIT = 900


def connect(device: str, sid: int):
    port = PortHandler(device)
    pkt = PacketHandler(PROTO)
    if not port.openPort():
        sys.exit(f"Cannot open {device}")
    if not port.setBaudRate(BAUD):
        sys.exit(f"Cannot set baud {BAUD}")
    model, res, _ = pkt.ping(port, sid)
    if res != COMM_SUCCESS:
        sys.exit("Servo not responding – check wiring & ID")
    pkt.write1ByteTxRx(port, sid, TORQUE_ENABLE, 1)
    return port, pkt, model


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default=DEVICE_DEFAULT)
    ap.add_argument("--id", type=int, default=SERVO_ID_DEFAULT)
    a = ap.parse_args()

    port, pkt, model = connect(a.device, a.id)
    cur, _, _ = pkt.read2ByteTxRx(port, a.id, PRESENT_POS_L)

    open_pos = OPEN_INIT
    close_pos = CLOSE_INIT

    print(f"Connected to ST3215 (model {model}) – current {cur} ticks")
    print("Enter a number 0-4095 to move, '[' to set OPEN, ']' to set CLOSE, 'q' to quit")

    try:
        while True:
            cmd = input("> ").strip().lower()
            if not cmd:
                continue
            if cmd == "q":
                break
            if cmd in "[]":
                if cmd == "[":
                    open_pos = cur
                    print(f"OPEN set to {open_pos} ticks")
                else:
                    close_pos = cur
                    print(f"CLOSE set to {close_pos} ticks")
                continue
            try:
                tgt = int(cmd)
            except ValueError:
                print("Please enter a number, '[' , ']' or 'q'")
                continue
            tgt = max(0, min(4095, tgt))
            pkt.write2ByteTxRx(port, a.id, GOAL_POS_L, tgt)
            time.sleep(0.3)
            cur, _, _ = pkt.read2ByteTxRx(port, a.id, PRESENT_POS_L)
            print(f"Reached {cur} ticks")
    finally:
        pkt.write1ByteTxRx(port, a.id, TORQUE_ENABLE, 0)
        port.closePort()

    print("\nCalibration complete:")
    print(f"OPEN  = {open_pos} ticks")
    print(f"CLOSE = {close_pos} ticks")
    print("Update these in /app/PiperGello/gello/robots/piper_robot.py and restart.")
    return 0


if __name__ == "__main__":
    sys.exit(main()) 