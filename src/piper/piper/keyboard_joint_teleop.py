#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Keyboard-based tele-operation publisher for the Piper arm.

Usage: started automatically by the launch file when *teleop_input:=keyboard*.
Publishes `sensor_msgs/JointState` on the `/joint_ctrl` topic.

Behaviour:
1.  Subscribes to `/joint_states` once at start-up; the first message seeds the
    internal command vector so the arm holds its current pose.
2.  A background thread reads single key presses using POSIX `termios` / `tty`.
    • Keys `1`-`7` select the active joint (1-6 = arm joints, 7 = gripper).
    • `+` or `=` increments, `-` decrements the selected joint by `±0.05`.
      For the gripper (joint 7) the value is clamped to *[0, 1]* (fully close →
      fully open).
    • `h` prints a short help message, `q` quits the node.
3.  The node republishes the current command at 10 Hz so the arm keeps its
    position until a new key press arrives.

This file intentionally avoids external dependencies – only the Python std-lib
and `rclpy`.*"""

import sys
import threading
import time
import select
import termios
import tty
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

STEP_SIZE = 0.05  # rad for joints 1-6, fraction for gripper (joint 7)
PUBLISH_RATE_HZ = 10.0


class KeyReader:
    """Non-blocking single-key reader (Unix only)."""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout: float = 0.1):
        """Return one character or *None* if nothing pressed within *timeout*."""
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            return ch
        return None


class KeyboardJointTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_joint_teleop')

        # Internal joint command buffer – will be seeded from /joint_states.
        self._cmd_positions: List[float] = [0.0] * 7
        self._positions_lock = threading.Lock()
        self._active_joint = 0  # 0-based index
        self._got_initial_state = threading.Event()

        # ROS interfaces
        self.create_subscription(JointState, 'joint_states', self._joint_states_cb, 10)
        self._pub = self.create_publisher(JointState, 'joint_ctrl', 1)
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_command)

        # Start keyboard thread
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        self.get_logger().info('Keyboard tele-operation node started. Press *h* for help.')

    # ------------------------------------------------------------------
    # Callbacks / threads
    # ------------------------------------------------------------------
    def _joint_states_cb(self, msg: JointState):
        if not self._got_initial_state.is_set():
            if len(msg.position) >= 6:
                with self._positions_lock:
                    # Take first 7 positions (gripper may be missing → default 0)
                    for i in range(7):
                        if i < len(msg.position):
                            self._cmd_positions[i] = msg.position[i]
                        else:
                            self._cmd_positions[i] = 0.0
                self._got_initial_state.set()
                self.get_logger().info('Initial joint positions captured.')

    def _publish_command(self):
        if not self._got_initial_state.is_set():
            return  # No command before we know starting pose

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        with self._positions_lock:
            js.position = list(self._cmd_positions)
        # No velocity / effort control from keyboard → leave empty
        self._pub.publish(js)

    # ------------------------------------------------------------------
    # Keyboard handling
    # ------------------------------------------------------------------
    def _keyboard_loop(self):
        help_text = (
            "\nKeyboard commands:\n"
            "  1-7 : select joint 1-7 (7 = gripper)\n"
            "  +/- : increase / decrease selected joint by ±0.05\n"
            "   h  : show this help\n"
            "   q  : quit node\n"
        )
        print(help_text)

        with KeyReader() as reader:
            while rclpy.ok():
                key = reader.read_key(0.1)
                if key is None:
                    continue

                if key in ('q', 'Q'):
                    print('\nExiting keyboard tele-op…')
                    rclpy.shutdown()
                    break

                if key in ('h', 'H', '?'):
                    print(help_text)
                    continue

                if key in '1234567':
                    self._active_joint = int(key) - 1
                    print(f'Active joint → {self._active_joint + 1}')
                    continue

                if key in ('+', '='):
                    self._update_active_joint(+STEP_SIZE)
                elif key == '-':
                    self._update_active_joint(-STEP_SIZE)

    def _update_active_joint(self, delta: float):
        with self._positions_lock:
            self._cmd_positions[self._active_joint] += delta
            # Clamp gripper (index 6) to [0,1]
            if self._active_joint == 6:
                self._cmd_positions[6] = max(0.0, min(1.0, self._cmd_positions[6]))
        joint_label = 'gripper' if self._active_joint == 6 else f'joint {self._active_joint + 1}'
        print(f'{joint_label} → {self._cmd_positions[self._active_joint]:.2f}')


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardJointTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 