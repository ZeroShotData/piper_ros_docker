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
import subprocess
from rclpy.qos import qos_profile_sensor_data
import rclpy.logging
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
import json
from datetime import datetime


class PiperRosNode(Node):
    """ROS2 node for the robotic arm"""

    def __init__(self, node_name: str = 'piper_ctrl_single_node', namespace: str = None) -> None:
        """Create a PiperRosNode instance.

        Args:
            node_name: Name of the node to register with the ROS graph. Defaults
                to ``'piper_ctrl_single_node'`` so behaviour is unchanged when
                used as a single-arm script.
            namespace: Optional ROS namespace under which all topics and
                services created by this node should live. When controlling
                multiple arms from the same machine you can run two instances
                of this class with e.g. ``namespace='arm1'`` and
                ``namespace='arm2'`` – all their topics will then be
                automatically isolated (``/arm1/joint_ctrl``,
                ``/arm2/joint_ctrl`` …).
        """
        super().__init__(node_name, namespace=namespace)
        # ROS parameters
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('gripper_exist', True)
        self.declare_parameter('gripper_val_mutiple', 1)
        self.declare_parameter('disable_gripper_auto_move', True)
        self.declare_parameter('rviz_ctrl_flag', False)
        self.declare_parameter('use_rosbridge', False)
        self.declare_parameter('operation_mode', 'teleop')
        self.declare_parameter('teleop_input', 'gello')
        # Monitor mode specific parameters
        self.declare_parameter('monitor_log_format', 'json')
        self.declare_parameter('monitor_rate_interval', 5.0)
        self.declare_parameter('monitor_topics', [])
        self.declare_parameter('monitor_source', 'lerobot')
        # Required gripper parameters published by loader
        self.declare_parameter('gripper/piper_gripper/open_ticks', Parameter.Type.INTEGER)
        self.declare_parameter('gripper/piper_gripper/close_ticks', Parameter.Type.INTEGER)

        self.can_port = self.get_parameter('can_port').get_parameter_value().string_value
        self.auto_enable = self.get_parameter('auto_enable').get_parameter_value().bool_value
        self.gripper_exist = self.get_parameter('gripper_exist').get_parameter_value().bool_value
        self.gripper_val_mutiple = self.get_parameter('gripper_val_mutiple').get_parameter_value().integer_value
        self.gripper_val_mutiple = max(0, min(self.gripper_val_mutiple, 10))
        self.disable_gripper_auto_move = self.get_parameter('disable_gripper_auto_move').get_parameter_value().bool_value
        self.rviz_ctrl_flag = self.get_parameter('rviz_ctrl_flag').get_parameter_value().bool_value
        self.use_rosbridge = self.get_parameter('use_rosbridge').get_parameter_value().bool_value
        self.operation_mode = self.get_parameter('operation_mode').get_parameter_value().string_value
        self.teleop_input = self.get_parameter('teleop_input').get_parameter_value().string_value
        # Monitor mode specific parameters
        self.monitor_log_format = self.get_parameter('monitor_log_format').get_parameter_value().string_value
        self.monitor_rate_interval = self.get_parameter('monitor_rate_interval').get_parameter_value().double_value
        self.monitor_topics = self.get_parameter('monitor_topics').get_parameter_value().string_array_value
        self.monitor_source = self.get_parameter('monitor_source').get_parameter_value().string_value
        
        # Validate operation mode
        if self.operation_mode not in ['teleop', 'replay', 'monitor']:
            self.get_logger().fatal(f'Invalid operation_mode: {self.operation_mode}. Must be "teleop", "replay", or "monitor"')
            raise SystemExit
        
        # Apply mode-specific parameter validation and overrides
        self._validate_and_configure_mode()

        # Initialize gripper parameters (conditional for monitor mode)
        self._initialize_gripper_parameters()

        self.get_logger().info(f"can_port is {self.can_port}")
        self.get_logger().info(f"auto_enable is {self.auto_enable}")
        self.get_logger().info(f"gripper_exist is {self.gripper_exist}")
        self.get_logger().info(f"gripper_val_mutiple is {self.gripper_val_mutiple}")
        self.get_logger().info(f"disable_gripper_auto_move is {self.disable_gripper_auto_move}")
        self.get_logger().info(f"rviz_ctrl_flag is {self.rviz_ctrl_flag}")
        self.get_logger().info(f"use_rosbridge is {self.use_rosbridge}")
        self.get_logger().info(f"operation_mode is {self.operation_mode}")
        if self.operation_mode == 'teleop':
            self.get_logger().info(f"teleop_input is {self.teleop_input}")
        if self.operation_mode == 'monitor':
            self.get_logger().info(f"monitor_log_format is {self.monitor_log_format}")
            self.get_logger().info(f"monitor_rate_interval is {self.monitor_rate_interval}")
            self.get_logger().info(f"monitor_topics is {self.monitor_topics}")
            self.get_logger().info(f"monitor_source is {self.monitor_source}")
        # keep console quiet to avoid timing stalls
        # self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
        
        # Mode-specific publisher creation to prevent conflicts
        self._validate_topic_publishers()
        if self.operation_mode == 'teleop':
            self._create_teleop_publishers()
        elif self.operation_mode == 'replay':
            self._create_replay_publishers()
        elif self.operation_mode == 'monitor':
            self._create_monitor_publishers()
        # Service
        self.motor_srv = self.create_service(Enable, 'enable_srv', self.handle_enable_service)
        # Joint
        self.joint_states = JointState()
        self.joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7
        
        # Joint ctrl message for storing control commands
        self.joint_ctrl = JointState()
        self.joint_ctrl.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_ctrl.position = [0.0] * 7
        self.joint_ctrl.velocity = [0.0] * 7
        self.joint_ctrl.effort = [0.0] * 7
        # Ensure attribute exists even when echo publisher is removed (standardized topic naming)
        self.joint_ctrl_pub = None
        
        # Enable flag
        self.__enable_flag = False
        
        # Conditional hardware initialization based on operation mode
        if self.operation_mode != 'monitor':
            # Hardware initialization only for teleop/replay modes
            self.get_logger().info(f"Initializing hardware interface for {self.operation_mode} mode")
            self.piper = C_PiperInterface(can_name=self.can_port)
            self.piper.ConnectPort()
            self.get_logger().info("Hardware interface initialized successfully")
        else:
            # Monitor mode: No hardware connection
            self.piper = None
            self.get_logger().info("Monitor mode: Hardware initialization skipped for safety")

        # QoS profile defined at top-level import for dropping old samples

        # Start subscription thread
        self.create_subscription(PosCmd, 'pos_cmd', self.pos_callback, 1)
        # Use sensor data QoS to ensure old commands are dropped when backend is slow
        self.create_subscription(JointState, 'joint_ctrl', self.joint_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, 'enable_flag', self.enable_callback, 1)

        # Buffer for the latest JointState command
        self._latest_joint_cmd = None
        self._joint_cmd_lock = threading.Lock()
        
        # Initialize rosbridge process handle
        self.rosbridge_process = None
        
        # Initialize monitor mode flag (default false for safety)
        self._monitor_mode_active = False
        
        # class-level, initialise with current pose or zeros
        self._last_cmd = {f'joint{i}': 0 for i in range(1, 7)}
        
        # Initialize simulated data for monitor mode BEFORE starting publisher thread
        if self.operation_mode == 'monitor':
            self._initialize_monitor_data_provider()
        
        # Setup mode-specific configuration
        self._setup_operation_mode()

        # Start publisher thread AFTER all initialization is complete
        self.publisher_thread = threading.Thread(target=self.publish_thread)
        self.publisher_thread.start()

    def _initialize_gripper_parameters(self):
        """Initialize gripper parameters only when needed"""
        if self.operation_mode == 'monitor':
            # Monitor mode: Use default values, no service dependency
            self.servo_open_ticks = 1450   # Default open position
            self.servo_close_ticks = 983   # Default closed position
            self.get_logger().info("Monitor mode: Using default gripper parameters")
            return
        
        # Teleop/replay modes: Use service as before
        client = self.create_client(GetParameters, '/gripper_config_loader/get_parameters')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().fatal('Loader parameter service unavailable')
            raise SystemExit
        req = GetParameters.Request(names=['gripper/piper_gripper/open_ticks', 'gripper/piper_gripper/close_ticks'])
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        results = future.result().values
        if not results or len(results) < 2:
            self.get_logger().fatal('Gripper ticks parameters not available')
            raise SystemExit
        self.servo_open_ticks = results[0].integer_value
        self.servo_close_ticks = results[1].integer_value

    def _has_hardware_connection(self):
        """Check if hardware interface is available"""
        return self.piper is not None

    def _initialize_monitor_data_provider(self):
        """Initialize simulated data provider for monitor mode"""
        self._simulated_joint_positions = [0.0] * 7  # Safe home position
        self._last_command_positions = [0.0] * 7    # Last received command
        self._simulated_joint_velocities = [0.0] * 7
        self._simulated_joint_efforts = [0.0] * 7
        self.get_logger().info("Monitor mode data provider initialized")

    def _validate_topic_publishers(self):
        """Validate no conflicting publishers exist before creating our own"""
        try:
            # Get list of topics that would conflict if we publish to them
            critical_topics = self._get_critical_topics_for_mode()
            
            if not critical_topics:
                return  # No critical topics for this mode
            
            # Check each critical topic for existing publishers
            for topic in critical_topics:
                topic_info = self.get_topic_names_and_types()
                publishers_info = self.get_publishers_info_by_topic(topic)
                
                if publishers_info:
                    # Found existing publishers - this is a conflict
                    self._handle_publisher_conflicts(topic, publishers_info)
                    
        except Exception as e:
            self.get_logger().warn(f"Topic validation failed: {e}")
            # Continue anyway for development, but log the warning
    
    def _get_critical_topics_for_mode(self):
        """Get list of topics that would conflict if we publish to them"""
        if self.operation_mode == 'teleop':
            # Teleop mode with Gello needs exclusive access to joint_ctrl
            # But with keyboard input, the keyboard node publishes to joint_ctrl
            if self.teleop_input == 'gello':
                return ['/joint_ctrl']
            else:  # keyboard
                return []  # No critical topics - keyboard node needs to publish
        elif self.operation_mode == 'replay':
            # Replay mode should NOT publish to joint_ctrl - RosBridge handles it
            return []
        elif self.operation_mode == 'monitor':
            # Monitor mode should NEVER publish control commands
            return []
        return []
    
    def _handle_publisher_conflicts(self, topic, publishers_info):
        """Handle detected publisher conflicts"""
        pub_count = len(publishers_info)
        
        error_msg = f"""
ERROR: Publisher conflict detected!

Topic: {topic}
Current Publishers: {pub_count}
"""
        
        for pub_info in publishers_info:
            error_msg += f"- {pub_info.node_name} (namespace: {pub_info.node_namespace})\n"
        
        error_msg += f"""
Resolution Options:
1. Stop conflicting nodes: ros2 node kill <node_name>
2. Use different namespace: --ros-args -r __ns:=/arm_unique
3. Switch to replay/monitor mode: operation_mode:=replay

Current mode: {self.operation_mode}
Cannot start in {self.operation_mode} mode with existing publishers on {topic}.
"""
        
        self.get_logger().fatal(error_msg)
        raise SystemExit(f"Publisher conflict on {topic}")
    
    def _create_teleop_publishers(self):
        """Create full publisher set for teleop mode (Gello control)"""
        self.get_logger().info("Creating teleop publishers (full set)")
        
        # Full publisher set - needs joint_ctrl_pub for Gello control
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, 'arm_status', 1)
        self.end_pose_pub = self.create_publisher(Pose, 'end_pose', 1)
        self.servo_cmd_pub = self.create_publisher(Int32, 'servo/command_raw', 1)
        
        self.get_logger().info("Teleop publishers created: joint_states, arm_status, end_pose, servo_cmd")
    
    def _create_replay_publishers(self):
        """Create limited publisher set for replay mode (NO joint_ctrl_pub)"""
        self.get_logger().info("Creating replay publishers (limited set - no joint_ctrl)")
        
        # Limited publisher set - NO joint_ctrl_pub (RosBridge publishes directly)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, 'arm_status', 1)
        self.end_pose_pub = self.create_publisher(Pose, 'end_pose', 1)
        self.servo_cmd_pub = self.create_publisher(Int32, 'servo/command_raw', 1)
        
        
        self.get_logger().info("Replay publishers created: joint_states, arm_status, end_pose, servo_cmd (NO joint_ctrl)")
    
    def _create_monitor_publishers(self):
        """Create minimal publisher set for monitor mode (NO joint_ctrl_pub or servo_cmd_pub)"""
        self.get_logger().info("Creating monitor publishers (minimal set - no control commands)")
        
        # Minimal publisher set - NO joint_ctrl_pub or servo_cmd_pub
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, 'arm_status', 1)
        self.end_pose_pub = self.create_publisher(Pose, 'end_pose', 1)
        
        # Set control publishers to None to indicate we don't publish to them
        self.servo_cmd_pub = None
        
        self.get_logger().info("Monitor publishers created: joint_states, arm_status, end_pose (NO control commands)")

    def _publish_simulated_joint_states(self):
        """Publish simulated joint states for monitor mode"""
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.position = self._simulated_joint_positions
        self.joint_states.velocity = self._simulated_joint_velocities
        self.joint_states.effort = self._simulated_joint_efforts
        self.joint_pub.publish(self.joint_states)
        
        # Monitor mode logging for joint states
        if self._is_monitor_mode_active():
            self._monitor_message_callback('/joint_states', self.joint_states)

    def _publish_simulated_arm_status(self):
        """Publish mock arm status for monitor mode"""
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = 1  # Normal mode
        arm_status.arm_status = 1  # Ready
        arm_status.mode_feedback = 1
        arm_status.teach_status = 0
        arm_status.motion_status = 0  # Stopped
        arm_status.trajectory_num = 0
        arm_status.err_code = 0  # No errors
        # Set all joint status to healthy (boolean values)
        arm_status.joint_1_angle_limit = False
        arm_status.joint_2_angle_limit = False
        arm_status.joint_3_angle_limit = False
        arm_status.joint_4_angle_limit = False
        arm_status.joint_5_angle_limit = False
        arm_status.joint_6_angle_limit = False
        arm_status.communication_status_joint_1 = True
        arm_status.communication_status_joint_2 = True
        arm_status.communication_status_joint_3 = True
        arm_status.communication_status_joint_4 = True
        arm_status.communication_status_joint_5 = True
        arm_status.communication_status_joint_6 = True
        self.arm_status_pub.publish(arm_status)
        
        # Monitor mode logging for arm status
        if self._is_monitor_mode_active():
            self._monitor_message_callback('/arm_status', arm_status)

    def _publish_simulated_end_pose(self):
        """Publish simulated end effector pose for monitor mode"""
        endpos = Pose()
        # Use default home position values
        endpos.position.x = 0.3  # 300mm forward
        endpos.position.y = 0.0  # Center
        endpos.position.z = 0.2  # 200mm above base
        # Default orientation (pointing down)
        endpos.orientation.x = 0.0
        endpos.orientation.y = 0.0
        endpos.orientation.z = 0.0
        endpos.orientation.w = 1.0
        self.end_pose_pub.publish(endpos)
        
        # Monitor mode logging for end pose
        if self._is_monitor_mode_active():
            self._monitor_message_callback('/end_pose', endpos)

    def _process_monitor_commands(self):
        """Process commands in monitor mode - logging only"""
        with self._joint_cmd_lock:
            joint_data = self._latest_joint_cmd
            self._latest_joint_cmd = None
        
        if joint_data is not None:
            # Log the command for monitoring
            self.get_logger().info(f"Monitor mode: Command received but not executed (safety)")
            # Update internal state for simulation
            self._update_simulated_state(joint_data)

    def _update_simulated_state(self, joint_data):
        """Update simulated state based on received commands"""
        if joint_data and len(joint_data.position) >= 6:
            # Update simulated positions (slowly move towards command)
            for i in range(min(6, len(joint_data.position))):
                target = joint_data.position[i]
                current = self._simulated_joint_positions[i]
                # Gradual movement towards target (10% per update)
                self._simulated_joint_positions[i] = current + 0.1 * (target - current)
            
            # Update gripper if available
            if len(joint_data.position) >= 7:
                target_gripper = joint_data.position[6]
                current_gripper = self._simulated_joint_positions[6]
                self._simulated_joint_positions[6] = current_gripper + 0.1 * (target_gripper - current_gripper)

    def _validate_and_configure_mode(self):
        """Validate and configure parameters based on operation mode"""
        if self.operation_mode == 'teleop':
            # Teleop mode should not use rosbridge by default
            if self.use_rosbridge:
                self.get_logger().warn("Teleop mode typically doesn't need rosbridge, but use_rosbridge=true")
        elif self.operation_mode == 'replay':
            # Replay mode requires rosbridge and should disable auto_enable
            if not self.use_rosbridge:
                self.get_logger().info("Replay mode requires rosbridge, enabling automatically")
                self.use_rosbridge = True
            if self.auto_enable:
                self.get_logger().info("Replay mode detected, disabling auto_enable")
                self.auto_enable = False
        elif self.operation_mode == 'monitor':
            # Monitor mode requires rosbridge and should disable auto_enable and gripper
            if not self.use_rosbridge:
                self.get_logger().info("Monitor mode requires rosbridge, enabling automatically")
                self.use_rosbridge = True
            if self.auto_enable:
                self.get_logger().info("Monitor mode detected, disabling auto_enable for safety")
                self.auto_enable = False
            # Validate monitor log format
            if self.monitor_log_format not in ['json', 'structured', 'simple', 'positions']:
                self.get_logger().warn(f"Invalid monitor_log_format: {self.monitor_log_format}, defaulting to 'json'")
                self.monitor_log_format = 'json'

    def _setup_operation_mode(self):
        """Setup mode-specific configuration"""
        if self.operation_mode == 'teleop':
            self._setup_teleop_mode()
        elif self.operation_mode == 'replay':
            self._setup_replay_mode()
        elif self.operation_mode == 'monitor':
            self._setup_monitor_mode()

    def _setup_teleop_mode(self):
        """Setup teleop mode configuration"""
        self.get_logger().info("Configuring node for teleop mode")
        
        # Start rosbridge server for teleop mode if enabled
        if self.use_rosbridge:
            self._start_rosbridge_server()

    def _setup_replay_mode(self):
        """Setup replay mode configuration"""
        self.get_logger().info("Configuring node for replay mode")
        
        # Start rosbridge server for replay mode
        if self.use_rosbridge:
            self._start_rosbridge_server()

    def _setup_monitor_mode(self):
        """Setup monitor mode configuration - pure monitoring, no robot control"""
        self.get_logger().info(f"Configuring node for monitor mode (source: {self.monitor_source})")
        
        # Initialize monitor mode flag for safety guards
        self._monitor_mode_active = True
        
        # Log mode-specific behavior
        if self.monitor_source == 'gello':
            self.get_logger().info("Monitor mode: Gello hardware monitoring enabled")
            self.get_logger().info("Monitor mode: Gello nodes will be launched for hardware input")
        else:  # lerobot
            self.get_logger().info("Monitor mode: LeRobot external monitoring enabled")
            self.get_logger().info("Monitor mode: Listening for external joint commands")
        
        # Start rosbridge server for monitor mode
        if self.use_rosbridge:
            self._start_rosbridge_server()
        
        # Create enhanced logging system
        self._create_monitor_logger()
        
        # Log safety warning
        self.get_logger().warn("Monitor mode active - all robot control commands will be blocked for safety")

    def _start_rosbridge_server(self):
        """Start the rosbridge server process with health check"""
        self.get_logger().info("Starting rosbridge server for replay mode...")
        
        # Always kill ALL existing rosbridge processes to ensure clean state
        self.get_logger().info("Cleaning up any existing rosbridge processes...")
        try:
            # Kill by multiple patterns to ensure we get everything
            subprocess.run(["pkill", "-f", "rosbridge_websocket"], capture_output=True)
            subprocess.run(["pkill", "-f", "rosbridge_server"], capture_output=True)
            time.sleep(1.5)  # Give more time for complete termination
            
            # Force kill if anything remains
            subprocess.run(["pkill", "-9", "-f", "rosbridge_websocket"], capture_output=True)
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().debug(f"Error during cleanup: {e}")
        
        # Start fresh rosbridge
        try:
            self.rosbridge_process = subprocess.Popen(
                ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # Health check: verify rosbridge is actually running
            if self._verify_rosbridge_health(timeout=15.0):
                self.get_logger().info("Rosbridge server started and verified successfully")
            else:
                self.get_logger().error("Rosbridge server started but health check failed")
                if self.rosbridge_process:
                    self.rosbridge_process.kill()
                    self.rosbridge_process = None
                
        except Exception as e:
            self.get_logger().error(f"Failed to start rosbridge server: {e}")
            self.rosbridge_process = None

    def _verify_rosbridge_health(self, timeout=10.0):
        """Verify rosbridge server is running and accessible
        
        Args:
            timeout: Maximum time to wait for rosbridge to become available
            
        Returns:
            bool: True if rosbridge is healthy, False otherwise
        """
        import socket
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                # Check if rosbridge websocket port (9090) is accessible
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                result = sock.connect_ex(('localhost', 9090))
                sock.close()
                
                if result == 0:
                    self.get_logger().info("Rosbridge health check passed - port 9090 is accessible")
                    # Additional check: ensure no stale publishers on critical topics
                    self._check_for_stale_publishers()
                    return True
                    
            except Exception as e:
                self.get_logger().debug(f"Rosbridge health check attempt failed: {e}")
            
            # Wait before retry
            time.sleep(0.5)
        
        self.get_logger().warn(f"Rosbridge health check timed out after {timeout}s")
        return False
    
    def _check_for_stale_publishers(self):
        """Check for stale publishers on critical topics and warn if found"""
        try:
            # Check if there are existing publishers on joint_ctrl
            publishers = self.get_publishers_info_by_topic('/joint_ctrl')
            if publishers:
                self.get_logger().warn(f"Found {len(publishers)} existing publishers on /joint_ctrl")
                for pub in publishers:
                    self.get_logger().warn(f"  - {pub.node_name} (namespace: {pub.node_namespace})")
                self.get_logger().warn("This may cause conflicts with LeRobot. Consider restarting if issues occur.")
        except Exception as e:
            self.get_logger().debug(f"Could not check for stale publishers: {e}")

    def _create_monitor_logger(self):
        """Create enhanced logging system for monitor mode"""
        # Initialize monitoring statistics
        self._message_stats = {}
        self._monitor_start_time = time.time()
        
        # Default monitored topics if none specified
        if not self.monitor_topics:
            self.monitor_topics = [
                '/joint_ctrl',
                '/joint_states',
                '/arm_status',
                '/end_pose'
            ]
        
        # Create monitoring subscriptions for joint control commands
        self._monitor_joint_sub = self.create_subscription(
            JointState, 
            'joint_ctrl', 
            lambda msg: self._monitor_message_callback('/joint_ctrl', msg),
            qos_profile_sensor_data
        )
        
        # Create timer for periodic statistics reporting
        self._stats_timer = self.create_timer(
            self.monitor_rate_interval,
            self._calculate_message_rates
        )
        
        # Initialize connection health monitoring
        self._connection_health = {
            'rosbridge_status': 'unknown',
            'clients_connected': 0,
            'last_health_check': 0,
            'connection_uptime': 0
        }
        
        # Create timer for connection health monitoring
        self._health_timer = self.create_timer(
            2.0,  # Check every 2 seconds
            self._monitor_connection_health
        )
        
        self.get_logger().info(f"Monitor mode logging configured:")
        self.get_logger().info(f"  Source: {self.monitor_source}")
        self.get_logger().info(f"  Format: {self.monitor_log_format}")
        self.get_logger().info(f"  Rate interval: {self.monitor_rate_interval}s")
        self.get_logger().info(f"  Monitored topics: {self.monitor_topics}")
        
        # Log initial monitor mode status (skip for positions format)
        if self.monitor_log_format != 'positions':
            print(f"\n=== MONITOR MODE STARTED ===")
            print(f"Source: {self.monitor_source}")
            print(f"Format: {self.monitor_log_format}")
            print(f"Topics: {self.monitor_topics}")
            print(f"Statistics interval: {self.monitor_rate_interval}s")
            print("All robot control commands are blocked for safety.")
            print("=" * 30)

    def _is_monitor_mode_active(self):
        """Check if monitor mode is active for safety guards"""
        return hasattr(self, '_monitor_mode_active') and self._monitor_mode_active

    def _block_robot_control(self, method_name):
        """Safety guard to prevent robot control in monitor mode"""
        if self._is_monitor_mode_active():
            self.get_logger().warn(f"Robot control method '{method_name}' blocked - monitor mode active")
            return True
        return False

    def _calculate_safe_message_rate(self, current_time, last_time, min_time_diff=0.001):
        """Calculate message rate with zero-division protection
        
        Args:
            current_time: Current timestamp
            last_time: Previous message timestamp  
            min_time_diff: Minimum time difference to consider (default 1ms)
            
        Returns:
            float: Safe rate calculation or 0 if time difference too small
        """
        time_diff = current_time - last_time
        
        if time_diff <= 0:
            # Identical or backwards timestamps
            return 0.0
        elif time_diff < min_time_diff:
            # Time difference too small for reliable rate calculation
            return 0.0
        else:
            return 1.0 / time_diff

    def _monitor_message_callback(self, topic_name, msg):
        """Fixed callback with proper statistics initialization"""
        if not self._is_monitor_mode_active():
            return
            
        current_time = time.time()
        
        # Initialize or update message statistics
        if topic_name not in self._message_stats:
            self._message_stats[topic_name] = {
                'count': 0,
                'last_time': 0,  # Start with 0 to avoid immediate rate calculation
                'rates': [],
                'first_message_time': current_time
            }
        
        stats = self._message_stats[topic_name]
        stats['count'] += 1
        
        # Safe rate calculation
        if stats['last_time'] > 0:  # Only calculate rate after first message
            rate = self._calculate_safe_message_rate(current_time, stats['last_time'])
            if rate > 0:  # Only add valid rates
                stats['rates'].append(rate)
                # Keep only recent rates for averaging
                if len(stats['rates']) > 10:
                    stats['rates'] = stats['rates'][-10:]
        
        stats['last_time'] = current_time
        
        # Format and log message based on log format
        self._log_monitored_message(topic_name, msg, stats)

    def _log_monitored_message(self, topic_name, msg, stats):
        """Log monitored message in specified format"""
        if self.monitor_log_format == 'json':
            self._log_json_format(topic_name, msg, stats)
        elif self.monitor_log_format == 'structured':
            self._log_structured_format(topic_name, msg, stats)
        elif self.monitor_log_format == 'positions':
            self._log_positions_format(topic_name, msg, stats)
        else:  # simple
            self._log_simple_format(topic_name, msg, stats)

    def _get_average_rate(self, rates_list, fallback=0.0):
        """Get average rate with edge case protection"""
        if not rates_list or len(rates_list) == 0:
            return fallback
        
        # Filter out invalid rates (zeros, infinities, negatives)
        valid_rates = [r for r in rates_list if r > 0 and r < float('inf')]
        
        if not valid_rates:
            return fallback
            
        return sum(valid_rates) / len(valid_rates)

    def _calculate_overall_rate(self, total_count, uptime, fallback=0.0):
        """Calculate overall rate with protection"""
        if uptime <= 0 or total_count <= 0:
            return fallback
        
        return total_count / uptime

    def _log_json_format(self, topic_name, msg, stats):
        """Log message in JSON format"""
        avg_rate = self._get_average_rate(stats['rates'][-5:], 0.0)
        connection_status = self._get_connection_status()
        
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "mode": "monitor",
            "monitor_source": self.monitor_source,
            "topic": topic_name,
            "message_count": stats['count'],
            "rate_hz": round(avg_rate, 2),
            "uptime": round(time.time() - self._monitor_start_time, 2),
            "connection_status": connection_status['rosbridge_status'],
            "clients_connected": connection_status['clients_connected'],
            "connection_uptime": connection_status['connection_uptime']
        }
        
        # Add message content based on type
        if hasattr(msg, 'position') and hasattr(msg.position, '__iter__'):  # JointState
            log_entry["message"] = {
                "positions": list(msg.position),
                "velocities": list(msg.velocity) if msg.velocity else [],
                "efforts": list(msg.effort) if msg.effort else []
            }
        elif hasattr(msg, 'position') and hasattr(msg.position, 'x'):  # Pose
            log_entry["message"] = {
                "position": {"x": msg.position.x, "y": msg.position.y, "z": msg.position.z},
                "orientation": {"x": msg.orientation.x, "y": msg.orientation.y, "z": msg.orientation.z, "w": msg.orientation.w}
            }
        else:
            log_entry["message"] = str(msg)
        
        print(json.dumps(log_entry))

    def _log_structured_format(self, topic_name, msg, stats):
        """Log message in structured human-readable format"""
        avg_rate = self._get_average_rate(stats['rates'][-5:], 0.0)
        uptime = time.time() - self._monitor_start_time
        
        print(f"[MONITOR] {datetime.now().strftime('%H:%M:%S.%f')[:-3]} | {topic_name}")
        print(f"  Count: {stats['count']} | Rate: {avg_rate:.1f} Hz | Uptime: {uptime:.1f}s")
        
        if hasattr(msg, 'position') and hasattr(msg.position, '__iter__'):  # JointState
            positions = [f"{p:.3f}" for p in msg.position[:6]]  # First 6 joints
            print(f"  Joints: [{', '.join(positions)}]")
        elif hasattr(msg, 'position') and hasattr(msg.position, 'x'):  # Pose
            print(f"  Position: ({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f})")
        elif hasattr(msg, 'ctrl_mode'):  # PiperStatusMsg
            print(f"  Status: ctrl_mode={msg.ctrl_mode}, arm_status={msg.arm_status}, err_code={msg.err_code}")

    def _log_simple_format(self, topic_name, msg, stats):
        """Log message in simple format"""
        avg_rate = self._get_average_rate(stats['rates'][-5:], 0.0)
        print(f"{topic_name}: #{stats['count']} @ {avg_rate:.1f} Hz")

    def _log_positions_format(self, topic_name, msg, stats):
        """Log message in positions-only format - simplest possible output"""
        # Only log joint commands, not simulated joint states
        if topic_name != '/joint_ctrl':
            return  # Only output actual commands, not simulated states
            
        # Extract joint positions (6 main joints only)
        positions = self._extract_joint_positions(msg)
        
        if positions is not None and len(positions) == 6:
            # Only output if positions are not all zeros (actual command received)
            if any(abs(pos) > 0.001 for pos in positions):  # Threshold to ignore near-zero values
                formatted = " ".join([f"{pos:.2f}" for pos in positions])
                print(formatted)

    def _extract_joint_positions(self, msg):
        """Extract and validate joint positions"""
        if hasattr(msg, 'position') and hasattr(msg.position, '__iter__'):
            positions = list(msg.position)[:6]  # First 6 joints only
            # Pad with zeros if needed, ensure exactly 6 values
            while len(positions) < 6:
                positions.append(0.0)
            return positions[:6]
        return None

    def _extract_pose_positions(self, msg):
        """Extract pose position for pose messages"""
        if hasattr(msg, 'position') and hasattr(msg.position, 'x'):
            return [msg.position.x, msg.position.y, msg.position.z]
        return None

    def _calculate_message_rates(self):
        """Calculate and log message rate statistics"""
        if not self._is_monitor_mode_active():
            return
        
        # Skip statistics output in positions mode
        if self.monitor_log_format == 'positions':
            return
            
        current_time = time.time()
        uptime = current_time - self._monitor_start_time
        connection_status = self._get_connection_status()
        
        print(f"\n=== MONITOR STATISTICS (Uptime: {uptime:.1f}s) ===")
        print(f"Connection Status: {connection_status['rosbridge_status']}")
        print(f"Clients Connected: {connection_status['clients_connected']}")
        print(f"Connection Uptime: {connection_status['connection_uptime']}s")
        print("-" * 30)
        
        for topic, stats in self._message_stats.items():
            avg_rate = self._get_average_rate(stats['rates'][-10:], 0.0)
            total_count = stats['count']
            overall_rate = self._calculate_overall_rate(total_count, uptime, 0.0)
            
            print(f"{topic}:")
            print(f"  Total Messages: {total_count}")
            print(f"  Recent Rate: {avg_rate:.2f} Hz")
            print(f"  Overall Rate: {overall_rate:.2f} Hz")
        print("=" * 50)

    def _monitor_connection_health(self):
        """Monitor rosbridge connection health and client count"""
        if not self._is_monitor_mode_active():
            return
        
        # Skip connection health output in positions mode
        if self.monitor_log_format == 'positions':
            return
            
        current_time = time.time()
        self._connection_health['last_health_check'] = current_time
        
        # Check rosbridge server health
        rosbridge_healthy = self._check_rosbridge_health()
        self._connection_health['rosbridge_status'] = 'healthy' if rosbridge_healthy else 'unhealthy'
        
        # Try to get client count (simplified - actual implementation would query rosbridge)
        # For now, estimate based on message activity
        client_count = self._estimate_client_connections()
        self._connection_health['clients_connected'] = client_count
        
        # Update connection uptime
        if rosbridge_healthy:
            if self._connection_health['connection_uptime'] == 0:
                self._connection_health['connection_uptime'] = current_time
        else:
            self._connection_health['connection_uptime'] = 0

    def _check_rosbridge_health(self):
        """Quick health check for rosbridge server"""
        try:
            import socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1.0)
            result = sock.connect_ex(('localhost', 9090))
            sock.close()
            return result == 0
        except Exception:
            return False

    def _estimate_client_connections(self):
        """Estimate number of connected clients based on message activity"""
        # Simplified estimation - in a real implementation, you'd query rosbridge
        # For now, assume 1 client if we're receiving messages regularly
        if '/joint_ctrl' in self._message_stats:
            stats = self._message_stats['/joint_ctrl']
            if stats['rates'] and len(stats['rates']) > 0:
                recent_rate = sum(stats['rates'][-3:]) / len(stats['rates'][-3:])
                return 1 if recent_rate > 0.1 else 0
        return 0

    def _get_connection_status(self):
        """Get current connection status for logging"""
        health = self._connection_health
        uptime = time.time() - health['connection_uptime'] if health['connection_uptime'] > 0 else 0
        
        return {
            'rosbridge_status': health['rosbridge_status'],
            'clients_connected': health['clients_connected'],
            'connection_uptime': round(uptime, 1)
        }

    def GetEnableFlag(self):
        # Debug: Print enable status when checked
        self.get_logger().debug(f"Enable flag checked, status: {self.__enable_flag}")
        return self.__enable_flag

    def publish_thread(self):
        """Mode-aware publish thread"""
        if self.operation_mode == 'monitor':
            rate = self.create_rate(5)  # 5 Hz for monitor mode
        else:
            rate = self.create_rate(100)  # 100 Hz for teleop/replay
        self.get_logger().info(f"Starting publish loop for {self.operation_mode} mode")
        
        # Move enable check here - run ONCE at startup like old code
        if self.operation_mode in ['teleop', 'replay']:
            enable_flag = False
            timeout = 5
            if not hasattr(self, '_hardware_start_time'):
                self._hardware_start_time = time.time()
            elapsed_time_flag = False
            
            if(self.auto_enable): 
                while not (enable_flag):
                    elapsed_time = time.time() - self._hardware_start_time
                    enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0, 1000, 0x01, 0)
                    if(enable_flag):
                        self.__enable_flag = True
                    # Check if the timeout has been exceeded
                    if elapsed_time > timeout:
                        print("Timeout....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    self.get_logger().info("Waiting for enable flag")
                    time.sleep(1)
                    pass
            if(elapsed_time_flag):
                print("Automatic enable timeout, exiting program")
                exit(0)
        
        while rclpy.ok():
            if self.operation_mode == 'monitor':
                self._publish_thread_monitor_mode()
            elif self.operation_mode in ['teleop', 'replay']:
                self._publish_thread_hardware_mode()
            else:
                self.get_logger().error(f"Unknown operation mode: {self.operation_mode}")
                break
            
            rate.sleep()

    def _publish_thread_monitor_mode(self):
        """Monitor mode publishing - no hardware access"""
        
        # Publish simulated joint states for visualization
        self._publish_simulated_joint_states()
        
        # Publish mock arm status for monitoring completeness
        self._publish_simulated_arm_status()
        
        # Publish simulated end pose
        self._publish_simulated_end_pose()
        
        # Process any incoming joint commands (for logging only)
        self._process_monitor_commands()

    def _publish_thread_hardware_mode(self):
        """Hardware mode publishing - requires robot connection"""
        
        if not self._has_hardware_connection():
            self.get_logger().error("Hardware connection required but not available")
            return

        # Existing hardware publishing logic - runs at 100Hz now!
        self.PublishArmState()
        self.PublishArmJointAndGripper()
        self._process_latest_joint_command()
        self.PublishArmEndPose()
        
        # Periodically log the current end pose (every 5 seconds)
        if not hasattr(self, '_last_pose_debug_time'):
            self._last_pose_debug_time = time.time()
        current_time = time.time()
        if current_time - self._last_pose_debug_time > 5.0:
            endpos_x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis / 1000000
            endpos_y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis / 1000000
            endpos_z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis / 1000000
            endpos_roll = self.piper.GetArmEndPoseMsgs().end_pose.RX_axis / 1000
            endpos_pitch = self.piper.GetArmEndPoseMsgs().end_pose.RY_axis / 1000
            endpos_yaw = self.piper.GetArmEndPoseMsgs().end_pose.RZ_axis / 1000
            self.get_logger().debug(f"Current end effector pose: x={endpos_x}, y={endpos_y}, z={endpos_z}, " +
                                   f"roll={endpos_roll}, pitch={endpos_pitch}, yaw={endpos_yaw}")
            self._last_pose_debug_time = current_time

    def PublishArmState(self):
        # Hardware validation for safety
        if not self._has_hardware_connection():
            self.get_logger().error("PublishArmState: Hardware connection required but not available")
            return
            
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
        
        # Monitor mode logging for arm status
        if self._is_monitor_mode_active():
            self._monitor_message_callback('/arm_status', arm_status)

    def PublishArmJointAndGripper(self):
        # Hardware validation for safety
        if not self._has_hardware_connection():
            self.get_logger().error("PublishArmJointAndGripper: Hardware connection required but not available")
            return
            
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
        # Gripper speed might not be reported at high rate; default to 0 if unavailable
        try:
            vel_6: float = self.piper.GetArmGripperMsgs().gripper_state.grippers_speed / 1000
        except AttributeError:
            vel_6 = 0.0
        effort_0:float = self.piper.GetArmHighSpdInfoMsgs().motor_1.effort/1000
        effort_1:float = self.piper.GetArmHighSpdInfoMsgs().motor_2.effort/1000
        effort_2:float = self.piper.GetArmHighSpdInfoMsgs().motor_3.effort/1000
        effort_3:float = self.piper.GetArmHighSpdInfoMsgs().motor_4.effort/1000
        effort_4:float = self.piper.GetArmHighSpdInfoMsgs().motor_5.effort/1000
        effort_5:float = self.piper.GetArmHighSpdInfoMsgs().motor_6.effort/1000
        effort_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort/1000
        self.joint_states.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]
        self.joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, vel_6]
        self.joint_states.effort = [effort_0, effort_1, effort_2, effort_3, effort_4, effort_5, effort_6]
        # 发布所有消息
        self.joint_pub.publish(self.joint_states)
        
        # Monitor mode logging for joint states
        if self._is_monitor_mode_active():
            self._monitor_message_callback('/joint_states', self.joint_states)


    def PublishArmEndPose(self):
        # Hardware validation for safety
        if not self._has_hardware_connection():
            self.get_logger().error("PublishArmEndPose: Hardware connection required but not available")
            return
            
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
        
        # Monitor mode logging for end pose
        if self._is_monitor_mode_active():
            self._monitor_message_callback('/end_pose', endpos)

    def pos_callback(self, pos_data):
        """Callback function for subscribing to the end effector pose

        Args:
            pos_data (): The position data
        """
        # Monitor mode safety guard
        if self._block_robot_control("pos_callback"):
            return
            
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
            self.get_logger().debug(f"Setting robot pose: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
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
        """Store the latest JointState command and return quickly.

        Heavy work is moved to the publish thread so that we always act on the
        newest command and avoid building up a backlog of callbacks when the
        incoming command rate is very high.
        """
        # Monitor mode logging (lightweight)
        if self._is_monitor_mode_active():
            self._monitor_message_callback('/joint_ctrl', joint_data)
        
        with self._joint_cmd_lock:
            # Only keep reference to the newest message; older ones will be GC-ed
            self._latest_joint_cmd = joint_data
        # Nothing else to do here – keeping callback lightweight ensures we can
        # keep up with high publish rates.

    # ---------------------------------------------------------------------
    # Helper used in the publish thread to actually send the command to the robot
    # ---------------------------------------------------------------------
    def _process_latest_joint_command(self):
        with self._joint_cmd_lock:
            joint_data = self._latest_joint_cmd
            # Reset buffer so we know whether we already consumed this command
            self._latest_joint_cmd = None

        if joint_data is None:
            return  # Nothing new to process
            
        # Monitor mode safety guard
        if self._block_robot_control("_process_latest_joint_command"):
            return


        factor = 57324.840764  # 1000*180/3.14

        for idx, joint_name in enumerate(joint_data.name):
            self._last_cmd[joint_name] = round(joint_data.position[idx] * factor)

        if not self.GetEnableFlag():
            return  # Robot not enabled – skip

        # Ensure the controller is in MIT direct-joint-servo mode exactly once
        if not hasattr(self, "_mit_mode_set"):
            try:
                # Put arm in CAN-joint control + MIT servo (direct position mode)
                self.piper.MotionCtrl_2(ctrl_mode=0x01,  # CAN command control
                                        move_mode=0x01,  # MOVE-J
                                        move_spd_rate_ctrl=100,
                                        is_mit_mode=0xAD)
                self._mit_mode_set = True
                self.get_logger().info("Switched arm to MIT servo mode")
            except Exception as e:
                self.get_logger().warn(f"Failed to set MIT mode: {e}")

        # Velocity control – choose an appropriate overall velocity
        if joint_data.velocity:
            all_zeros = all(v == 0 for v in joint_data.velocity)
        else:
            all_zeros = True

        if not all_zeros:
            if len(joint_data.velocity) == 7:
                vel_all = clip(round(joint_data.velocity[6]), 1, 100)
                self.get_logger().debug(f"vel_all: {vel_all}")
                self.piper.MotionCtrl_2(0x01, 0x01, vel_all, 0xAD)
            else:
                self.get_logger().debug(f"vel_all not given")
                self.piper.MotionCtrl_2(0x01, 0x01, 30, 0xAD)
        else:
            self.piper.MotionCtrl_2(0x01, 0x01, 30, 0xAD)

        # Throttle logging of the processed joint command to once per second to aid debugging
        now = time.time()
        if not hasattr(self, "_last_cmd_debug_time"):
            self._last_cmd_debug_time = 0.0
        if now - self._last_cmd_debug_time > 1.0:
            self.get_logger().info(
                f"Joint cmd: j1={self._last_cmd['joint1']/57324.84:.2f} j2={self._last_cmd['joint2']/57324.84:.2f} "
                f"j3={self._last_cmd['joint3']/57324.84:.2f} j4={self._last_cmd['joint4']/57324.84:.2f} "
                f"j5={self._last_cmd['joint5']/57324.84:.2f} j6={self._last_cmd['joint6']/57324.84:.2f} rad"
            )
            self._last_cmd_debug_time = now
        else:
            self.get_logger().debug(
                "Setting joint angles: "
                + f"j1={self._last_cmd['joint1']}, "
                + f"j2={self._last_cmd['joint2']}, "
                + f"j3={self._last_cmd['joint3']}, "
                + f"j4={self._last_cmd['joint4']}, "
                + f"j5={self._last_cmd['joint5']}, "
                + f"j6={self._last_cmd['joint6']}"
            )

        # Now transmit the desired joint positions to the arm
        self.piper.JointCtrl(
            self._last_cmd['joint1'],
            self._last_cmd['joint2'],
            self._last_cmd['joint3'],
            self._last_cmd['joint4'],
            self._last_cmd['joint5'],
            self._last_cmd['joint6'],
        )

        # Gripper control (7th joint)
        if self.gripper_exist and len(joint_data.position) >= 7:
            # Convert gripper value (0-1) to servo ticks
            # Assuming open=1592 ticks, close=900 ticks (from calibration)
            SERVO_OPEN_TICKS = self.servo_open_ticks
            SERVO_CLOSE_TICKS = self.servo_close_ticks
            
            # joint_data.position[6] is in range 0-1 (normalized gripper value)
            gripper_normalized = joint_data.position[6]
            
            # self.get_logger().debug(f"Gripper normalized value: {gripper_normalized}")  # muted
            
            # Map 0-1 to servo ticks (0=open, 1=closed)
            # Reversed mapping: 0 -> CLOSE, 1 -> OPEN
            servo_ticks = int(SERVO_CLOSE_TICKS + (SERVO_OPEN_TICKS - SERVO_CLOSE_TICKS) * gripper_normalized)
            
            # self.get_logger().debug(f"Publishing servo ticks: {servo_ticks}")  # muted
            
            # Publish to servo (only if servo_cmd_pub exists - not available in monitor mode)
            if self.servo_cmd_pub is not None:
                servo_msg = Int32()
                servo_msg.data = servo_ticks
                self.servo_cmd_pub.publish(servo_msg)
            else:
                self.get_logger().debug(f"Skipping servo publish - not available in {self.operation_mode} mode")
            
            # Also use the CAN gripper if available (for compatibility)
            if len(joint_data.effort) >= 7:
                gripper_effort = clip(joint_data.effort[6], 0.5, 3)
                if not math.isnan(gripper_effort):
                    gripper_effort = round(gripper_effort * 1000)
                else:
                    gripper_effort = 0
                self.piper.GripperCtrl(abs(self._last_cmd['joint6']), gripper_effort, 0x01, 0)
            else:
                self.piper.GripperCtrl(abs(self._last_cmd['joint6']), 1000, 0x01, 0)

    def enable_callback(self, enable_flag: Bool):
        """Callback function for enabling the robotic arm

        Args:
            enable_flag (): Boolean flag
        """
        # Monitor mode safety guard
        if self._block_robot_control("enable_callback"):
            return
            
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
        # Monitor mode safety guard
        if self._block_robot_control("handle_enable_service"):
            resp.enable_response = False
            return resp
            
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


            self.get_logger().info(f"Sleeping for enable service")
            time.sleep(0.5)

        resp.enable_response = enable_flag
        self.get_logger().info(f"Returning response: {resp.enable_response}")
        return resp

    def destroy_node(self):
        # Terminate rosbridge server if it was started
        if self.rosbridge_process is not None:
            self.get_logger().info("Terminating rosbridge server...")
            self.rosbridge_process.terminate()
            try:
                self.rosbridge_process.wait(timeout=2)
                self.get_logger().info("Rosbridge server terminated")
            except subprocess.TimeoutExpired:
                self.get_logger().warning("Rosbridge server termination timed out, forcing kill")
                self.rosbridge_process.kill()
        
        # Always kill ALL rosbridge processes to ensure clean state for next run
        self.get_logger().info("Final cleanup of all rosbridge processes...")
        try:
            subprocess.run(["pkill", "-f", "rosbridge_websocket"], capture_output=True)
            subprocess.run(["pkill", "-f", "rosbridge_server"], capture_output=True) 
            time.sleep(0.5)
            # Force kill anything remaining
            subprocess.run(["pkill", "-9", "-f", "rosbridge_websocket"], capture_output=True)
        except Exception:
            pass  # Ignore errors during cleanup
        
        super().destroy_node()


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
