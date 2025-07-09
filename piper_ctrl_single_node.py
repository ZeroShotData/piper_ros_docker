#!/usr/bin/env python3
# -*-coding:utf8-*-
# This file controls a single robotic arm node and handles the movement of the robotic arm with a gripper.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32
import time
import threading
import argparse
import math
from piper_sdk import *
from piper_sdk import C_PiperInterface
from piper_msgs.msg import PiperStatusMsg, PosCmd
from piper_msgs.srv import Enable
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R  # For Euler angle to quaternion conversion
from numpy import clip


class PiperRosNode(Node):
    """ROS2 node for the robotic arm"""

    def __init__(self) -> None:
        super().__init__('piper_ctrl_single_node')
        # ROS parameters
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('gripper_exist', True)
        self.declare_parameter('gripper_val_mutiple', 1)

        self.can_port = self.get_parameter('can_port').get_parameter_value().string_value
        self.auto_enable = self.get_parameter('auto_enable').get_parameter_value().bool_value
        self.gripper_exist = self.get_parameter('gripper_exist').get_parameter_value().bool_value
        self.gripper_val_mutiple = self.get_parameter('gripper_val_mutiple').get_parameter_value().integer_value
        self.gripper_val_mutiple = max(0, min(self.gripper_val_mutiple, 10))

        self.get_logger().info(f"can_port is {self.can_port}")
        self.get_logger().info(f"auto_enable is {self.auto_enable}")
        self.get_logger().info(f"gripper_exist is {self.gripper_exist}")
        self.get_logger().info(f"gripper_val_mutiple is {self.gripper_val_mutiple}")
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states_single', 1)
        self.joint_ctrl_pub = self.create_publisher(JointState, 'joint_ctrl', 1)
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, 'arm_status', 1)
        self.end_pose_pub = self.create_publisher(Pose, 'end_pose', 1)
        # Publisher for ST3215 servo
        self.servo_cmd_pub = self.create_publisher(Int32, 'servo/command_raw', 1)
        # Service
        self.motor_srv = self.create_service(Enable, 'enable_srv', self.handle_enable_service)
        # Joint
        self.joint_states = JointState()
        self.joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7
        # Joint ctrl
        self.joint_ctrl = JointState()
        self.joint_ctrl.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_ctrl.position = [0.0] * 7
        self.joint_ctrl.velocity = [0.0] * 7
        self.joint_ctrl.effort = [0.0] * 7
        # Enable flag
        self.__enable_flag = False
        # Create piper class and open CAN interface
        self.piper = C_PiperInterface(can_name=self.can_port)
        self.piper.ConnectPort()

        # Start subscription thread
        self.create_subscription(PosCmd, 'pos_cmd', self.pos_callback, 1)
        self.create_subscription(JointState, 'joint_ctrl_single', self.joint_callback, 1)
        self.create_subscription(Bool, 'enable_flag', self.enable_callback, 1)

        self.publisher_thread = threading.Thread(target=self.publish_thread)
        self.publisher_thread.start()

    def GetEnableFlag(self):
        # Debug: Print enable status when checked
        self.get_logger().debug(f"Enable flag checked, status: {self.__enable_flag}")
        return self.__enable_flag

    def publish_thread(self):
        """Publish messages from the robotic arm
        """
        rate = self.create_rate(200)  # 200 Hz
        enable_flag = False
        # Set timeout (seconds)
        timeout = 5
        # Record the time before entering the loop
        start_time = time.time()
        elapsed_time_flag = False
        # Main publish loop
        self.get_logger().info("Starting publish loop")
        last_pose_debug_time = time.time()
        
        while rclpy.ok():
            if(self.auto_enable):
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    print("--------------------")
                    enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    print("Enable status:", enable_flag)
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0, 1000, 0x01, 0)
                    if(enable_flag):
                        self.__enable_flag = True
                    print("--------------------")
                    # Check if the timeout has been exceeded
                    if elapsed_time > timeout:
                        print("Timeout....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    pass
            if(elapsed_time_flag):
                print("Automatic enable timeout, exiting program")
                exit(0)

            # Publish all required data
            self.PublishArmState()
            self.PublishArmJointAndGripper()
            self.PublishArmCtrlAndGripper()
            self.PublishArmEndPose()
            
            # Periodically log the current end pose (every 5 seconds)
            current_time = time.time()
            if current_time - last_pose_debug_time > 5.0:
                endpos_x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis / 1000000
                endpos_y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis / 1000000
                endpos_z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis / 1000000
                endpos_roll = self.piper.GetArmEndPoseMsgs().end_pose.RX_axis / 1000
                endpos_pitch = self.piper.GetArmEndPoseMsgs().end_pose.RY_axis / 1000
                endpos_yaw = self.piper.GetArmEndPoseMsgs().end_pose.RZ_axis / 1000
                self.get_logger().debug(f"Current end effector pose: x={endpos_x}, y={endpos_y}, z={endpos_z}, " +
                                       f"roll={endpos_roll}, pitch={endpos_pitch}, yaw={endpos_yaw}")
                last_pose_debug_time = current_time
            
            # Sleep at the specified rate
            rate.sleep()

    def PublishArmState(self):
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)

    def PublishArmJointAndGripper(self):
        # Assign timestamp
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        # Here, you can set the joint positions to any value you want
        # The raw data obtained is in degrees multiplied by 1000. To convert to radians, divide by 1000, multiply by π/180, and limit to 5 decimal places
        joint_0: float = (self.piper.GetArmJointMsgs().joint_state.joint_1 / 1000) * 0.017444
        joint_1: float = (self.piper.GetArmJointMsgs().joint_state.joint_2 / 1000) * 0.017444
        joint_2: float = (self.piper.GetArmJointMsgs().joint_state.joint_3 / 1000) * 0.017444
        joint_3: float = (self.piper.GetArmJointMsgs().joint_state.joint_4 / 1000) * 0.017444
        joint_4: float = (self.piper.GetArmJointMsgs().joint_state.joint_5 / 1000) * 0.017444
        joint_5: float = (self.piper.GetArmJointMsgs().joint_state.joint_6 / 1000) * 0.017444
        joint_6: float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1000000
        vel_0: float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed / 1000
        vel_1: float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed / 1000
        vel_2: float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed / 1000
        vel_3: float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed / 1000
        vel_4: float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed / 1000
        vel_5: float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed / 1000
        effort_0:float = self.piper.GetArmHighSpdInfoMsgs().motor_1.effort/1000
        effort_1:float = self.piper.GetArmHighSpdInfoMsgs().motor_2.effort/1000
        effort_2:float = self.piper.GetArmHighSpdInfoMsgs().motor_3.effort/1000
        effort_3:float = self.piper.GetArmHighSpdInfoMsgs().motor_4.effort/1000
        effort_4:float = self.piper.GetArmHighSpdInfoMsgs().motor_5.effort/1000
        effort_5:float = self.piper.GetArmHighSpdInfoMsgs().motor_6.effort/1000
        effort_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort/1000
        self.joint_states.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]
        self.joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5]
        self.joint_states.effort = [effort_0, effort_1, effort_2, effort_3, effort_4, effort_5, effort_6]
        # 发布所有消息
        self.joint_pub.publish(self.joint_states)

    def PublishArmCtrlAndGripper(self):
        self.joint_ctrl.header.stamp = self.get_clock().now().to_msg()
        joint_0: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_1/1000) * 0.017444
        joint_1: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_2/1000) * 0.017444
        joint_2: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_3/1000) * 0.017444
        joint_3: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_4/1000) * 0.017444
        joint_4: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_5/1000) * 0.017444
        joint_5: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_6/1000) * 0.017444
        joint_6: float = self.piper.GetArmGripperCtrl().gripper_ctrl.grippers_angle/1000000
        self.joint_ctrl.position = [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]  # Example values
        self.joint_ctrl_pub.publish(self.joint_ctrl)

    def PublishArmEndPose(self):
        # End effector pose
        endpos = Pose()
        endpos.position.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis / 1000000
        endpos.position.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis / 1000000
        endpos.position.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis / 1000000
        roll = self.piper.GetArmEndPoseMsgs().end_pose.RX_axis / 1000
        pitch = self.piper.GetArmEndPoseMsgs().end_pose.RY_axis / 1000
        yaw = self.piper.GetArmEndPoseMsgs().end_pose.RZ_axis / 1000
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        quaternion = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        endpos.orientation.x = quaternion[0]
        endpos.orientation.y = quaternion[1]
        endpos.orientation.z = quaternion[2]
        endpos.orientation.w = quaternion[3]
        self.end_pose_pub.publish(endpos)

    def pos_callback(self, pos_data):
        """Callback function for subscribing to the end effector pose

        Args:
            pos_data (): The position data
        """
        factor = 180 / 3.1415926
        self.get_logger().info(f"Received PosCmd:")
        self.get_logger().info(f"x: {pos_data.x}")
        self.get_logger().info(f"y: {pos_data.y}")
        self.get_logger().info(f"z: {pos_data.z}")
        self.get_logger().info(f"roll: {pos_data.roll}")
        self.get_logger().info(f"pitch: {pos_data.pitch}")
        self.get_logger().info(f"yaw: {pos_data.yaw}")
        self.get_logger().info(f"gripper: {pos_data.gripper}")
        self.get_logger().info(f"mode1: {pos_data.mode1}")
        self.get_logger().info(f"mode2: {pos_data.mode2}")
        x = round(pos_data.x*1000) * 1000
        y = round(pos_data.y*1000) * 1000
        z = round(pos_data.z*1000) * 1000
        rx = round(pos_data.roll*1000*factor)
        ry = round(pos_data.pitch*1000*factor)
        rz = round(pos_data.yaw*1000*factor)
        if(self.GetEnableFlag()):
            self.get_logger().info(f"Setting robot pose: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
            self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
            self.piper.MotionCtrl_2(0x01, 0x02, 50)
            self.piper.EndPoseCtrl(x, y, z, rx, ry, rz)
            gripper = round(pos_data.gripper * 1000 * 1000)
            if pos_data.gripper > 80000:
                gripper = 80000
            if pos_data.gripper < 0:
                gripper = 0
            if self.gripper_exist:
                self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)
            self.piper.MotionCtrl_2(0x01, 0x00, 50)
        else:
            self.get_logger().warn(f"Cannot set pose: Robot not enabled")

    def joint_callback(self, joint_data):
        """Callback function for joint angles

        Args:
            joint_data (): The joint data
        """
        self.get_logger().info("--- JOINT_CALLBACK TRIGGERED ---")
        factor = 57324.840764  # 1000*180/3.14
        self.get_logger().info(f"Received Joint States:")

        # 创建一个字典来存储关节名称与位置的映射
        joint_positions = {}
        joint_6 = 0

        # 遍历joint_data.name来映射位置
        for idx, joint_name in enumerate(joint_data.name):
            self.get_logger().info(f"{joint_name}: {joint_data.position[idx]}")
            joint_positions[joint_name] = round(joint_data.position[idx] * factor)
        
        # 获取第7个关节的位置
        if len(joint_data.position) >= 7:
            self.get_logger().info(f"gripper: {joint_data.position[6]}")
            joint_6 = round(joint_data.position[6] * 1000 * 1000)
            joint_6 = joint_6 * self.gripper_val_mutiple

        # 控制电机速度
        if self.GetEnableFlag():
            if joint_data.velocity != []:
                all_zeros = all(v == 0 for v in joint_data.velocity)
            else:
                all_zeros = True
            if not all_zeros:
                lens = len(joint_data.velocity)
                if lens == 7:
                    vel_all = clip(round(joint_data.velocity[6]), 1, 100)
                    self.get_logger().debug(f"vel_all: {vel_all}")
                    self.piper.MotionCtrl_2(0x01, 0x01, vel_all)
                else:
                    self.get_logger().debug(f"vel_all not given")
                    self.piper.MotionCtrl_2(0x01, 0x01, 30)
            else:
                self.piper.MotionCtrl_2(0x01, 0x01, 30)

            # 使用关节名称来动态控制关节
            self.get_logger().info(f"Setting joint angles: " +
                                  f"j1={joint_positions.get('joint1', 0)}, " +
                                  f"j2={joint_positions.get('joint2', 0)}, " +
                                  f"j3={joint_positions.get('joint3', 0)}, " +
                                  f"j4={joint_positions.get('joint4', 0)}, " +
                                  f"j5={joint_positions.get('joint5', 0)}, " +
                                  f"j6={joint_positions.get('joint6', 0)}")
            self.piper.JointCtrl(
                joint_positions.get('joint1', 0),
                joint_positions.get('joint2', 0),
                joint_positions.get('joint3', 0),
                joint_positions.get('joint4', 0),
                joint_positions.get('joint5', 0),
                joint_positions.get('joint6', 0)
            )

            # 夹爪控制
            if self.gripper_exist:
                # Convert gripper value (0-1) to servo ticks
                # Assuming open=1592 ticks, close=842 ticks (from calibration)
                SERVO_OPEN_TICKS = 1592
                SERVO_CLOSE_TICKS = 842
                
                # joint_6 is in range 0-1 (normalized gripper value)
                gripper_normalized = joint_data.position[6] if len(joint_data.position) >= 7 else 0.0
                
                # self.get_logger().info(f"Gripper normalized value: {gripper_normalized}")  # muted – too verbose
                
                # Map 0-1 to servo ticks (0=open, 1=closed)
                # Invert mapping: 0 -> CLOSE, 1 -> OPEN
                servo_ticks = int(SERVO_CLOSE_TICKS + (SERVO_OPEN_TICKS - SERVO_CLOSE_TICKS) * gripper_normalized)
                
                # self.get_logger().info(f"Publishing servo ticks: {servo_ticks}")  # muted – too verbose
                
                # Publish to servo
                servo_msg = Int32()
                servo_msg.data = servo_ticks
                self.servo_cmd_pub.publish(servo_msg)
                
                # Also use the CAN gripper if available (for compatibility)
                if len(joint_data.effort) >= 7:
                    gripper_effort = clip(joint_data.effort[6], 0.5, 3)
                    # self.get_logger().info(f"gripper_effort: {gripper_effort}")
                    if not math.isnan(gripper_effort):
                        gripper_effort = round(gripper_effort * 1000)
                    else:
                        # self.get_logger().warning("Gripper effort is NaN, using default value.")
                        gripper_effort = 0  # 设置默认值
                    self.piper.GripperCtrl(abs(joint_6), gripper_effort, 0x01, 0)
                else:
                    self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)


    def enable_callback(self, enable_flag: Bool):
        """Callback function for enabling the robotic arm

        Args:
            enable_flag (): Boolean flag
        """
        self.get_logger().info(f"Received enable flag:")
        self.get_logger().info(f"enable_flag: {enable_flag.data}")
        if enable_flag.data:
            self.__enable_flag = True
            self.get_logger().debug("Enabling robotic arm")
            self.piper.EnableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
        else:
            self.__enable_flag = False
            self.get_logger().debug("Disabling robotic arm")
            self.piper.DisableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x00, 0)

    def handle_enable_service(self, req, resp):
        """Handle enable service for the robotic arm"""
        self.get_logger().info(f"Received request: {req.enable_request}")
        enable_flag = False
        loop_flag = False
        # Set timeout duration (seconds)
        timeout = 5
        # Record the time before entering the loop
        start_time = time.time()
        while not loop_flag:
            elapsed_time = time.time() - start_time
            self.get_logger().info(f"--------------------")
            enable_list = []
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status)

            self.get_logger().debug(f"Enable status list: {enable_list}")
            if req.enable_request:
                enable_flag = all(enable_list)
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
            else:
                enable_flag = any(enable_list)
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x02, 0)

            self.get_logger().info(f"Enable status: {enable_flag}")
            self.__enable_flag = enable_flag
            self.get_logger().info(f"--------------------")

            if enable_flag == req.enable_request:
                loop_flag = True
                enable_flag = True
            else:
                loop_flag = False
                enable_flag = False

            # Check if timeout duration has been exceeded
            if elapsed_time > timeout:
                self.get_logger().info(f"Timeout...")
                enable_flag = False
                loop_flag = True
                break

            time.sleep(0.5)

        resp.enable_response = enable_flag
        self.get_logger().info(f"Returning response: {resp.enable_response}")
        return resp


def main(args=None):
    rclpy.init(args=args)
    piper_single_node = PiperRosNode()
    try:
        rclpy.spin(piper_single_node)
    except KeyboardInterrupt:
        pass
    finally:
        piper_single_node.destroy_node()
        rclpy.shutdown()
