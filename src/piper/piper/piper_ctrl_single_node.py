#!/usr/bin/env python3
# -*-coding:utf8-*-
# This file controls a single robotic arm node and handles the movement of the robotic arm with a gripper.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import threading
import argparse
import math
import socket
import json
from piper_sdk import *
from piper_sdk import C_PiperInterface
from piper_msgs.msg import PiperStatusMsg, PosCmd
from piper_msgs.srv import Enable
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R  # For Euler angle to quaternion conversion
from numpy import clip
import select  # For non-blocking socket operations


class PiperRosNode(Node):
    """ROS2 node for the robotic arm"""

    def __init__(self) -> None:
        super().__init__('piper_ctrl_single_node')
        # ROS parameters
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('gripper_exist', True)
        self.declare_parameter('gripper_val_mutiple', 1)
        self.declare_parameter('rviz_ctrl_flag', False)
        self.declare_parameter('listen_port', 0)  # 0 means don't listen

        self.can_port = self.get_parameter('can_port').get_parameter_value().string_value
        self.auto_enable = self.get_parameter('auto_enable').get_parameter_value().bool_value
        self.gripper_exist = self.get_parameter('gripper_exist').get_parameter_value().bool_value
        self.gripper_val_mutiple = self.get_parameter('gripper_val_mutiple').get_parameter_value().integer_value
        self.gripper_val_mutiple = max(0, min(self.gripper_val_mutiple, 10))
        self.rviz_ctrl_flag = self.get_parameter('rviz_ctrl_flag').get_parameter_value().bool_value
        self.listen_port = self.get_parameter('listen_port').get_parameter_value().integer_value

        self.get_logger().info(f"can_port is {self.can_port}")
        self.get_logger().info(f"auto_enable is {self.auto_enable}")
        self.get_logger().info(f"gripper_exist is {self.gripper_exist}")
        self.get_logger().info(f"gripper_val_mutiple is {self.gripper_val_mutiple}")
        self.get_logger().info(f"rviz_ctrl_flag is {self.rviz_ctrl_flag}")
        self.get_logger().info(f"listen_port is {self.listen_port}")
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states_single', 1)
        self.joint_ctrl_pub = self.create_publisher(JointState, 'joint_ctrl', 1)
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, 'arm_status', 1)
        self.end_pose_pub = self.create_publisher(Pose, 'end_pose', 1)
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

        # TCP server for remote control
        self.end_effector_pose = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
        }
        
        # Store the latest command instead of using a queue
        self.latest_commands = {}  # Dictionary to store latest command for each client
        self.client_sockets = []
        self.command_lock = threading.Lock()  # Lock to synchronize access to latest_commands
        
        # Start subscription thread
        self.create_subscription(PosCmd, 'pos_cmd', self.pos_callback, 1)
        self.create_subscription(JointState, 'joint_ctrl_single', self.joint_callback, 1)
        self.create_subscription(Bool, 'enable_flag', self.enable_callback, 1)

        self.publisher_thread = threading.Thread(target=self.publish_thread)
        self.publisher_thread.start()
        
        # Start TCP server if port is specified
        if self.listen_port > 0:
            self.get_logger().info(f"Starting TCP server on port {self.listen_port}")
            self.server_thread = threading.Thread(target=self.tcp_server_thread)
            self.server_thread.daemon = True
            self.server_thread.start()

    def tcp_server_thread(self):
        """TCP server thread for remote control of the robotic arm"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.settimeout(0.1)  # Make accept non-blocking with short timeout
        
        try:
            server_socket.bind(('0.0.0.0', self.listen_port))
            server_socket.listen(5)
            self.get_logger().info(f"TCP server listening on port {self.listen_port}")
            
            while rclpy.ok():
                try:
                    # Use select for non-blocking accept
                    readable, _, _ = select.select([server_socket], [], [], 0.1)
                    if server_socket in readable:
                        client_socket, addr = server_socket.accept()
                        self.get_logger().info(f"Connection from {addr}")
                        self.client_sockets.append(client_socket)
                        client_thread = threading.Thread(
                            target=self.handle_client,
                            args=(client_socket, addr)
                        )
                        client_thread.daemon = True
                        client_thread.start()
                except Exception as e:
                    self.get_logger().error(f"Error accepting connection: {e}")
                    
                # Process latest commands for all clients
                self.process_latest_commands()
                time.sleep(0.01)  # Small sleep to prevent CPU hogging
        except Exception as e:
            self.get_logger().error(f"Error starting TCP server: {e}")
        finally:
            server_socket.close()
            for sock in self.client_sockets:
                try:
                    sock.close()
                except:
                    pass
    
    def process_latest_commands(self):
        """Process the latest command for each client"""
        with self.command_lock:
            # Make a copy to avoid modifying while iterating
            commands_to_process = list(self.latest_commands.items())
            # Clear the commands dictionary
            self.latest_commands.clear()
            
        for client_id, (cmd, client_socket) in commands_to_process:
            try:
                if cmd.get('command') == 'get_pose':
                    # Get current end effector pose - just use the cached value
                    response = json.dumps(self.end_effector_pose) + '\n'
                    client_socket.sendall(response.encode('utf-8'))
                elif cmd.get('command') == 'set_pose':
                    # Set end effector pose
                    if not self.GetEnableFlag():
                        response = {'status': 'error', 'message': 'Robot not enabled'}
                    else:
                        pos_data = PosCmd()
                        pos_data.x = float(cmd.get('x', self.end_effector_pose['x']))
                        pos_data.y = float(cmd.get('y', self.end_effector_pose['y']))
                        pos_data.z = float(cmd.get('z', self.end_effector_pose['z']))
                        pos_data.roll = float(cmd.get('roll', self.end_effector_pose['roll']))
                        pos_data.pitch = float(cmd.get('pitch', self.end_effector_pose['pitch']))
                        pos_data.yaw = float(cmd.get('yaw', self.end_effector_pose['yaw']))
                        pos_data.gripper = float(cmd.get('gripper', 0.0))
                        pos_data.mode1 = 0
                        pos_data.mode2 = 0
                        self.pos_callback(pos_data)
                        response = {'status': 'ok'}
                    client_socket.sendall((json.dumps(response) + '\n').encode('utf-8'))
                else:
                    response = {'status': 'error', 'message': 'Unknown command'}
                    client_socket.sendall((json.dumps(response) + '\n').encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Error processing command: {e}")
    
    def handle_client(self, client_socket, addr):
        """Handle client connection"""
        # Set socket to non-blocking
        client_socket.setblocking(0)
        buffer = ""
        client_id = id(client_socket)  # Unique identifier for this client
        
        try:
            while rclpy.ok():
                try:
                    # Use select for non-blocking receive
                    readable, _, _ = select.select([client_socket], [], [], 0.1)
                    if client_socket in readable:
                        data = client_socket.recv(1024)
                        if not data:
                            break
                        
                        # Add to buffer and process complete messages
                        buffer += data.decode('utf-8')
                        messages = buffer.split('\n')
                        
                        # Process all complete messages - keep only the latest one
                        latest_cmd = None
                        for i in range(len(messages) - 1):
                            try:
                                latest_cmd = json.loads(messages[i])
                            except json.JSONDecodeError:
                                response = {'status': 'error', 'message': 'Invalid JSON'}
                                client_socket.sendall((json.dumps(response) + '\n').encode('utf-8'))
                        
                        # Store only the latest command
                        if latest_cmd:
                            with self.command_lock:
                                self.latest_commands[client_id] = (latest_cmd, client_socket)
                        
                        # Keep the incomplete message in the buffer
                        buffer = messages[-1]
                except socket.error as e:
                    if e.args[0] == socket.EWOULDBLOCK:
                        # No data available, continue
                        continue
                    else:
                        # Actual error
                        self.get_logger().error(f"Socket error: {e}")
                        break
                except Exception as e:
                    self.get_logger().error(f"Error handling client: {e}")
                    break
                
                time.sleep(0.01)  # Small sleep to prevent CPU hogging
        except Exception as e:
            self.get_logger().error(f"Error in client handler: {e}")
        finally:
            try:
                if client_socket in self.client_sockets:
                    self.client_sockets.remove(client_socket)
                # Remove any stored commands for this client
                with self.command_lock:
                    if client_id in self.latest_commands:
                        del self.latest_commands[client_id]
                client_socket.close()
            except:
                pass
            self.get_logger().info(f"Connection closed from {addr}")

    def GetEnableFlag(self):
        return self.__enable_flag

    def publish_thread(self):
        """Publish messages from the robotic arm"""
        rate = self.create_rate(200)  # 200 Hz
        
        # Handle auto-enable with cleaner code
        if self.auto_enable:
            self.get_logger().info("Auto-enable is active, attempting to enable arm...")
            start_time = time.time()
            timeout = 5
            
            while rclpy.ok():
                # Check all motors at once
                motor_states = [
                    self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status,
                    self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status,
                    self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status,
                    self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status,
                    self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status,
                    self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                ]
                
                all_enabled = all(motor_states)
                if all_enabled:
                    self.get_logger().info("All motors enabled successfully")
                    self.__enable_flag = True
                    break
                
                # Check for timeout
                if (time.time() - start_time) > timeout:
                    self.get_logger().error("Timeout while enabling motors, exiting")
                    exit(0)
                
                # Try enabling and wait a bit
                self.piper.EnableArm(7)
                if self.gripper_exist:
                    self.piper.GripperCtrl(0, 1000, 0x01, 0)
                time.sleep(0.5)
                
                # Periodically report status (once per second)
                if int(time.time() - start_time) != int(time.time() - start_time - 0.1):
                    enabled_count = sum(1 for state in motor_states if state)
                    self.get_logger().info(f"Enabling motors: {enabled_count}/6 enabled")
        
        # Main publish loop
        self.get_logger().info("Starting publish loop")
        while rclpy.ok():
            # Publish all required data
            self.PublishArmState()
            self.PublishArmJointAndGripper()
            self.PublishArmCtrlAndGripper()
            self.PublishArmEndPose()
            
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
        
        # Update stored end effector pose (cache it for fast retrieval)
        self.end_effector_pose = {
            'x': endpos.position.x,
            'y': endpos.position.y,
            'z': endpos.position.z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }

    def pos_callback(self, pos_data):
        """Callback function for subscribing to the end effector pose

        Args:
            pos_data (): The position data
        """
        factor = 180 / 3.1415926
        # Only log at debug level and less frequently
        if self.get_clock().now().nanoseconds % 1000000000 < 50000000:  # Log roughly every second
            self.get_logger().debug(f"Received PosCmd: x={pos_data.x}, y={pos_data.y}, z={pos_data.z}, " +
                                   f"rpy=[{pos_data.roll},{pos_data.pitch},{pos_data.yaw}], gripper={pos_data.gripper}")
        
        # Convert values with minimal processing
        x = round(pos_data.x*1000) * 1000
        y = round(pos_data.y*1000) * 1000
        z = round(pos_data.z*1000) * 1000
        rx = round(pos_data.roll*1000*factor)
        ry = round(pos_data.pitch*1000*factor)
        rz = round(pos_data.yaw*1000*factor)
        
        if(self.GetEnableFlag()):
            self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
            self.piper.MotionCtrl_2(0x01, 0x02, 50)
            self.piper.EndPoseCtrl(x, y, z, rx, ry, rz)
            gripper = round(pos_data.gripper * 1000 * 1000)
            gripper = max(0, min(gripper, 80000))  # Clamp values efficiently
            
            if self.gripper_exist:
                self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)
            self.piper.MotionCtrl_2(0x01, 0x00, 50)

    def joint_callback(self, joint_data):
        """Callback function for joint angles

        Args:
            joint_data (): The joint data
        """
        if not self.GetEnableFlag():
            return  # Skip processing if not enabled

        factor = 57324.840764  # 1000*180/3.14
        
        # Create a joint positions dictionary more efficiently
        joint_positions = {name: round(joint_data.position[idx] * factor) 
                          for idx, name in enumerate(joint_data.name) if idx < len(joint_data.position)}
        
        # Get gripper value if available
        joint_6 = 0
        if len(joint_data.position) >= 7:
            joint_6 = round(joint_data.position[6] * 1000 * 1000 * self.gripper_val_mutiple)
            
        # Control motor speed
        if joint_data.velocity and not all(v == 0 for v in joint_data.velocity):
            if len(joint_data.velocity) == 7:
                vel_all = clip(round(joint_data.velocity[6]), 1, 100)
                self.piper.MotionCtrl_2(0x01, 0x01, vel_all)
            else:
                self.piper.MotionCtrl_2(0x01, 0x01, 30)
        else:
            self.piper.MotionCtrl_2(0x01, 0x01, 30)

        # Control joints more efficiently
        self.piper.JointCtrl(
            joint_positions.get('joint1', 0),
            joint_positions.get('joint2', 0),
            joint_positions.get('joint3', 0),
            joint_positions.get('joint4', 0),
            joint_positions.get('joint5', 0),
            joint_positions.get('joint6', 0)
        )

        # Gripper control
        if self.gripper_exist:
            if len(joint_data.effort) >= 7:
                gripper_effort = clip(joint_data.effort[6], 0.5, 3)
                if not math.isnan(gripper_effort):
                    gripper_effort = round(gripper_effort * 1000)
                else:
                    gripper_effort = 1000  # Default value
                self.piper.GripperCtrl(abs(joint_6), gripper_effort, 0x01, 0)
            else:
                self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)

    def enable_callback(self, enable_flag: Bool):
        """Callback function for enabling the robotic arm

        Args:
            enable_flag (): Boolean flag
        """
        if self.__enable_flag == enable_flag.data:
            return  # Skip if no change
            
        self.get_logger().info(f"Setting enable flag to: {enable_flag.data}")
        
        if enable_flag.data:
            self.__enable_flag = True
            self.piper.EnableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
        else:
            self.__enable_flag = False
            self.piper.DisableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x00, 0)

    def handle_enable_service(self, req, resp):
        """Handle enable service for the robotic arm"""
        self.get_logger().info(f"Received enable service request: {req.enable_request}")
        
        # Initialize variables
        enable_flag = False
        # Set timeout duration (seconds)
        timeout = 5.0
        start_time = time.time()
        
        # Fast polling with less logging
        poll_interval = 0.1  # Poll more frequently for better responsiveness
        while (time.time() - start_time) < timeout:
            # Get current status once for all motors
            enable_list = [
                self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
            ]
            
            # Update status based on request
            if req.enable_request:
                # We want to enable, check if all motors are enabled
                enable_flag = all(enable_list)
                self.piper.EnableArm(7)
                if self.gripper_exist:
                    self.piper.GripperCtrl(0, 1000, 0x01, 0)
            else:
                # We want to disable, check if any motor is still enabled
                enable_flag = not any(enable_list)
                self.piper.DisableArm(7)
                if self.gripper_exist:
                    self.piper.GripperCtrl(0, 1000, 0x02, 0)
            
            # Check if we're in the desired state
            if (req.enable_request and enable_flag) or (not req.enable_request and enable_flag):
                self.__enable_flag = req.enable_request
                break
                
            # Small sleep to prevent CPU hogging
            time.sleep(poll_interval)
        
        # Service response
        resp.enable_response = enable_flag
        self.get_logger().info(f"Enable service response: {resp.enable_response}")
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
