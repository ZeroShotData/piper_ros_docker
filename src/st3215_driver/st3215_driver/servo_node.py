#!/usr/bin/env python3
"""
ROS 2 driver for a single Feetech ST3215 servo *in raw ticks*.

Topics
------
/servo/command_raw   std_msgs/Int32   ──> desired position in ticks
/servo/position_raw  std_msgs/Int32   <── current position in ticks
/joint_states        sensor_msgs/JointState (optional, still published)

Services (optional)
/servo/torque_enable std_srvs/SetBool
"""
import os, rclpy, time
from rclpy.node         import Node
from std_msgs.msg       import Int32
from sensor_msgs.msg    import JointState
from std_srvs.srv       import SetBool
from scservo_sdk        import PortHandler, PacketHandler, COMM_SUCCESS

# ---------------- user-adjustable constants ----------------
SERVO_ID      = 6
DEVICE        = os.getenv("SERVO_DEVICE", "/dev/ttyACM0")
BAUDRATE      = 1_000_000

GOAL_POS_L    = 42
PRESENT_POS_L = 56
TORQUE_ENABLE = 40
PROTOCOL_END  = 0      # ST3215 = protocol 0
# -----------------------------------------------------------

class ST3215Driver(Node):
    def __init__(self):
        super().__init__('st3215_driver')

        # ROS interfaces
        self.cmd_sub  = self.create_subscription(
            Int32, 'servo/command_raw', self.cmd_cb, 10)
        self.pos_pub  = self.create_publisher(
            Int32, 'servo/position_raw', 10)
        self.js_pub   = self.create_publisher(
            JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.read_and_publish)   # 20 Hz

        self.torque_srv = self.create_service(
            SetBool, 'servo/torque_enable', self.torque_cb)

        # Feetech SDK
        self.port = PortHandler(DEVICE)
        self.pkt  = PacketHandler(PROTOCOL_END)
        if not self.port.openPort():
            self.get_logger().fatal(f'Cannot open {DEVICE}')
            raise SystemExit
        if not self.port.setBaudRate(BAUDRATE):
            self.get_logger().fatal(f'Cannot set baud {BAUDRATE}')
            raise SystemExit

        model, res, _ = self.pkt.ping(self.port, SERVO_ID)
        if res != COMM_SUCCESS:
            raise RuntimeError('Servo not responding')

        self.enable_torque(True)
        self.get_logger().info(f'ST3215 ready (model {model})')

    # ---------------- helpers --------------------------------
    def enable_torque(self, en: bool) -> bool:
        res, _ = self.pkt.write1ByteTxRx(
            self.port, SERVO_ID, TORQUE_ENABLE, 1 if en else 0)
        return res == COMM_SUCCESS

    # ---------------- ROS callbacks ---------------------------
    def cmd_cb(self, msg: Int32):
        self.pkt.write2ByteTxRx(self.port, SERVO_ID, GOAL_POS_L, msg.data)

    def torque_cb(self, req, resp):
        resp.success = self.enable_torque(req.data)
        return resp

    def read_and_publish(self):
        pos, res, _ = self.pkt.read2ByteTxRx(self.port, SERVO_ID, PRESENT_POS_L)
        if res != COMM_SUCCESS:
            return

        # raw position topic
        self.pos_pub.publish(Int32(data=pos))

        # standard JointState (ticks in position field so RViz still works)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['st3215_joint']
        js.position = [float(pos)]   # still ticks, not radians
        self.js_pub.publish(js)


def main():
    rclpy.init()
    node = ST3215Driver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
