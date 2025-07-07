#!/usr/bin/env python3
"""Replay test logger

Subscribes to /joint_ctrl and periodically prints out the number
of messages received together with an estimated frequency.  Intended to
verify connectivity between the capture computer (publishing via
rosbridge-websocket) and the Piper ROS 2 environment.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32


class ReplayLogger(Node):
    def __init__(self):
        super().__init__('replay_logger')
        # Depth 10 queue, default reliability (best effort OK)
        self._sub = self.create_subscription(
            JointState,
            '/joint_ctrl',
            self._callback,
            10,
        )
        # Publisher to echo back received messages for testing
        self._pub = self.create_publisher(
            JointState,
            '/joint_states',
            10,
        )
        # Publisher for servo position feedback
        self._servo_pub = self.create_publisher(
            Int32,
            '/servo/position_raw',
            10,
        )
        self._msg_count = 0
        self._start_time = self.get_clock().now().nanoseconds  # int64 ns
        self._logged_once = False
        
        # Timer to publish default joint states for connection testing
        self._timer = self.create_timer(0.1, self._publish_default_joints)

    # ------------------------------------------------------------------
    def _callback(self, msg: JointState):
        self._msg_count += 1
        
        # Log every message received from lerobot
        pos_pretty = ', '.join(f"{p:.2f}" for p in msg.position)
        self.get_logger().info(f"CMD #{self._msg_count}: [{pos_pretty}]")
        
        # Echo back the received message as joint states for testing
        echo_msg = JointState()
        echo_msg.header.stamp = self.get_clock().now().to_msg()
        echo_msg.name = msg.name
        echo_msg.position = msg.position
        echo_msg.velocity = msg.velocity if msg.velocity else [0.0] * len(msg.position)
        echo_msg.effort = msg.effort if msg.effort else [0.0] * len(msg.position)
        self._pub.publish(echo_msg)
        
        # Every 30 messages (~1 s at 30 Hz) print rate stats
        if self._msg_count % 30 == 0:
            now_ns = self.get_clock().now().nanoseconds
            elapsed_sec = (now_ns - self._start_time) / 1e9
            hz = self._msg_count / elapsed_sec if elapsed_sec else 0.0
            self.get_logger().info(f"STATS: {self._msg_count} msgs | {hz:.2f} Hz")

    def _publish_default_joints(self):
        # Always publish default joint states to simulate robot feedback
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._pub.publish(msg)
        
        # Publish servo position feedback (default closed position)
        servo_msg = Int32()
        servo_msg.data = 983  # Default gripper closed position in ticks
        self._servo_pub.publish(servo_msg)
        
        if not self._logged_once:
            self.get_logger().info("Publishing joint states and servo feedback for lerobot connection test")
            self._logged_once = True


# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ReplayLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 