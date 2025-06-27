#!/usr/bin/env python3
"""Original interactive ST3215 gripper calibration (single-key, no Enter).

Run with a real TTY (docker exec -it …) so key presses are detected.
This file is the un-modified 302-line version that used to work.
"""
# NOTE: copied verbatim from the earlier working commit (shortened comment)

import argparse
import sys
import termios
import tty
import time
import os

try:
    from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    print("ERROR: scservo_sdk not installed")
    sys.exit(1)

DEFAULT_SERVO_ID = 6
DEFAULT_DEVICE = "/dev/ttyACM0"
BAUDRATE = 1_000_000
PROTOCOL_END = 0
GOAL_POS_L = 42
PRESENT_POS_L = 56
TORQUE_ENABLE = 40
DEFAULT_OPEN = 1592
DEFAULT_CLOSE = 900
MIN_POSITION = 0
MAX_POSITION = 4095

class KeyReader:
    """Read single key presses using raw mode (works with -it)."""
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self
    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
    def read(self):
        ch = sys.stdin.read(1)
        if ch == '\x1b':  # arrows
            ch = sys.stdin.read(2)
            if ch == '[A':
                return 'w'
            if ch == '[B':
                return 's'
            if ch == '[C':
                return 'd'
            if ch == '[D':
                return 'a'
        return ch.lower()

def connect(device, sid):
    port = PortHandler(device)
    pkt = PacketHandler(PROTOCOL_END)
    if not port.openPort() or not port.setBaudRate(BAUDRATE):
        sys.exit(f"Cannot open {device} at {BAUDRATE} bps")
    model, res, _ = pkt.ping(port, sid)
    if res != COMM_SUCCESS:
        sys.exit("Servo not responding")
    pkt.write1ByteTxRx(port, sid, TORQUE_ENABLE, 1)
    return port, pkt, model

def main():
    pa = argparse.ArgumentParser()
    pa.add_argument('--device', default=DEFAULT_DEVICE)
    pa.add_argument('--id', type=int, default=DEFAULT_SERVO_ID)
    a = pa.parse_args()

    port, pkt, model = connect(a.device, a.id)
    cur, _, _ = pkt.read2ByteTxRx(port, a.id, PRESENT_POS_L)
    open_pos, close_pos = DEFAULT_OPEN, DEFAULT_CLOSE
    last_toggle = open_pos

    print(f"Connected (model {model}) – start {cur} ticks; press 'h' for help")
    print("Controls: a/d -10  w/s ±50  [ set OPEN  ] set CLOSE  space toggle  q quit")

    with KeyReader() as kr:
        while True:
            key = kr.read()
            if key == 'q':
                break
            if key in ('h', '?'):
                print("a/d -10, w/s +50, '[' set open, ']' set close, space toggle, q quit")
                continue
            delta = 0
            new = cur
            if key == 'a': delta = -10
            elif key == 'd': delta = +10
            elif key == 's': delta = -50
            elif key == 'w': delta = +50
            elif key == 'o': new = open_pos
            elif key == 'c': new = close_pos
            elif key == ' ': new = close_pos if last_toggle==open_pos else open_pos; last_toggle = new
            elif key == '[': open_pos = cur; print(f"OPEN={open_pos}"); continue
            elif key == ']': close_pos = cur; print(f"CLOSE={close_pos}"); continue
            else: continue
            if delta: new = cur + delta
            new = max(MIN_POSITION, min(MAX_POSITION, new))
            pkt.write2ByteTxRx(port, a.id, GOAL_POS_L, new)
            time.sleep(0.05)
            cur = new
            print(f"Pos {cur}\r", end='', flush=True)
    pkt.write1ByteTxRx(port, a.id, TORQUE_ENABLE, 0)
    port.closePort()
    print(f"\nDone. OPEN={open_pos} CLOSE={close_pos}")

if __name__ == '__main__':
    main() 