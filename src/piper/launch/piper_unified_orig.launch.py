"""Unified Piper Robot Launch File

This consolidated launch file replaces the three separate launch files:
- start_piper.launch.py (teleop mode)
- replay_mode.launch.py (replay mode)  
- websocket_logger.launch.py (websocket server)

Usage:
  Teleop Mode:
    ros2 launch piper piper_unified.launch.py operation_mode:=teleop gripper_config:=/app/configs/my_setup.yaml
  
  Replay Mode:
    ros2 launch piper piper_unified.launch.py operation_mode:=replay gripper_config:=/app/configs/my_setup.yaml
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # -----------------------
    # Launch arguments
    # -----------------------
    operation_mode_arg = DeclareLaunchArgument(
        'operation_mode',
        default_value='teleop',
        choices=['teleop', 'replay', 'monitor'],
        description='Operation mode: teleop (with Gello), replay (websocket control), or monitor (pure monitoring)'
    )

    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port used by the Piper controller.'
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='',  # Will be set based on mode
        description='Automatically enable the Piper controller.'
    )

    rviz_ctrl_flag_arg = DeclareLaunchArgument(
        'rviz_ctrl_flag',
        default_value='false',
        description='Launch rviz visualisation.'
    )

    gello_exist_arg = DeclareLaunchArgument(
        'gello_exist',
        default_value='',  # Will be set based on mode
        description='Whether the Gello bridge is running.'
    )

    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='Whether a gripper is attached.'
    )

    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='1',
        description='Scalar applied to gripper values.'
    )

    disable_gripper_auto_move_arg = DeclareLaunchArgument(
        'disable_gripper_auto_move',
        default_value='true',
        description='Prevent gripper from moving to zero at startup.'
    )

    use_rosbridge_arg = DeclareLaunchArgument(
        'use_rosbridge',
        default_value='',  # Will be set based on mode
        description='Start rosbridge server for remote access.'
    )

    # Gripper YAML configuration (required, no default)
    gripper_config_arg = DeclareLaunchArgument(
        'gripper_config',
        description='Absolute path to gripper YAML configuration.'
    )
    
    # Monitor mode specific arguments
    monitor_log_format_arg = DeclareLaunchArgument(
        'monitor_log_format',
        default_value='json',
        choices=['json', 'structured', 'simple', 'positions'],
        description='Monitor mode log format: json (full data), structured (human-readable), simple (counts), positions (joint values only)'
    )
    
    monitor_rate_interval_arg = DeclareLaunchArgument(
        'monitor_rate_interval',
        default_value='5.0',
        description='Monitor mode statistics reporting interval in seconds'
    )

    # -----------------------
    # Mode-specific configuration
    # -----------------------
    def configure_mode_parameters(context, *args, **kwargs):
        """Configure parameters based on operation mode"""
        mode = context.launch_configurations.get('operation_mode', 'teleop')
        
        # Set mode-specific defaults if not explicitly provided
        if not context.launch_configurations.get('auto_enable'):
            if mode == 'teleop':
                context.launch_configurations['auto_enable'] = 'true'
            elif mode == 'replay':
                context.launch_configurations['auto_enable'] = 'false'
            else:  # monitor
                context.launch_configurations['auto_enable'] = 'false'
        
        if not context.launch_configurations.get('gello_exist'):
            if mode == 'teleop':
                context.launch_configurations['gello_exist'] = 'true'
            elif mode == 'replay':
                context.launch_configurations['gello_exist'] = 'false'
            else:  # monitor
                context.launch_configurations['gello_exist'] = 'false'
        
        if not context.launch_configurations.get('use_rosbridge'):
            if mode == 'teleop':
                context.launch_configurations['use_rosbridge'] = 'true'
            elif mode == 'replay':
                context.launch_configurations['use_rosbridge'] = 'true'
            else:  # monitor
                context.launch_configurations['use_rosbridge'] = 'true'
        
        # Monitor mode specific overrides
        if mode == 'monitor':
            if not context.launch_configurations.get('gripper_exist'):
                context.launch_configurations['gripper_exist'] = 'false'
        
        # Sync gripper with gello (from old start_piper.launch.py logic)
        if context.launch_configurations.get('gello_exist', 'true').lower() == 'false':
            # Force gripper to be disabled when gello bridge is disabled
            context.launch_configurations['gripper_exist'] = 'false'
            # Also turn off rosbridge unless the user explicitly overrides later
            if context.launch_configurations.get('use_rosbridge', 'true').lower() != 'false':
                context.launch_configurations['use_rosbridge'] = 'false'
        
        return []

    mode_config = OpaqueFunction(function=configure_mode_parameters)

    # -----------------------
    # Hardware Processes (conditional)
    # -----------------------
    def create_hardware_processes(context, *args, **kwargs):
        """Create hardware processes only when needed"""
        mode = context.launch_configurations.get('operation_mode', 'teleop')
        if mode != 'monitor':
            # CAN interface activation process
            can_activate_proc = ExecuteProcess(
                cmd=['bash', '/app/can_activate.sh', 'can0', '1000000', '1-1.1:1.0'],
                name='activate_can',
                output='screen'
            )

            # Ensure expected serial-by-id path exists by linking to /dev/ttyUSB0
            create_serial_link = ExecuteProcess(
                name='create_serial_symlink',
                output='screen',
                cmd=[
                    'bash', '-c',
                    (
                        'TARGET=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NMKV-if00-port0; '
                        '[ -e "$TARGET" ] || { '
                        'mkdir -p /dev/serial/by-id && ln -sf /dev/ttyUSB0 "$TARGET" && '
                        'echo "Created $TARGET -> /dev/ttyUSB0"; }'
                    )
                ]
            )
            return [can_activate_proc, create_serial_link]
        else:
            # Monitor mode: Skip hardware setup
            return [ExecuteProcess(
                cmd=['echo', 'Monitor mode: Skipping CAN and hardware setup for safety'],
                name='monitor_mode_info',
                output='screen'
            )]

    hardware_processes = OpaqueFunction(function=create_hardware_processes)

    # -----------------------
    # Core Nodes (conditional)
    # -----------------------
    def create_core_nodes(context, *args, **kwargs):
        """Create core nodes based on operation mode"""
        mode = context.launch_configurations.get('operation_mode', 'teleop')
        
        if mode == 'monitor':
            # Monitor mode: Only essential monitoring components
            nodes = [
                Node(
                    package='piper',
                    executable='piper_single_ctrl', 
                    name='piper_ctrl_single_node',
                    output='screen',
                    parameters=[{
                        'operation_mode': 'monitor',
                        'can_port': LaunchConfiguration('can_port'),
                        'auto_enable': LaunchConfiguration('auto_enable'),
                        'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple'),
                        'gripper_exist': LaunchConfiguration('gripper_exist'),
                        'disable_gripper_auto_move': LaunchConfiguration('disable_gripper_auto_move'),
                        'rviz_ctrl_flag': LaunchConfiguration('rviz_ctrl_flag'),
                        'use_rosbridge': LaunchConfiguration('use_rosbridge'),
                        'monitor_log_format': LaunchConfiguration('monitor_log_format'),
                        'monitor_rate_interval': LaunchConfiguration('monitor_rate_interval'),
                        # Skip hardware-related parameters
                    }],
                    remappings=[('joint_states_single', 'joint_states')],
                )
            ]
            
            # Only add replay logger for non-positions formats
            log_format = context.launch_configurations.get('monitor_log_format', 'json')
            if log_format != 'positions':
                nodes.append(Node(
                    package='piper', 
                    executable='piper_replay_logger',
                    name='replay_logger',
                    output='screen'
                ))
            
            return nodes
        else:
            # Full hardware setup for teleop/replay
            loader_node = Node(
                package='gripper_config_loader',
                executable='gripper_config_loader',
                name='gripper_config_loader',
                output='screen',
                parameters=[{
                    'gripper_config': LaunchConfiguration('gripper_config'),
                    'gello_exist': LaunchConfiguration('gello_exist')
                }]
            )
            
            servo_node = Node(
                package='st3215_driver',
                executable='st3215_servo',
                name='st3215_servo',
                output='screen',
                parameters=[],
                condition=IfCondition(LaunchConfiguration('gripper_exist'))
            )
            
            piper_node = Node(
                package='piper',
                executable='piper_single_ctrl',
                name='piper_ctrl_single_node',
                output='screen',
                parameters=[{
                    'can_port': LaunchConfiguration('can_port'),
                    'auto_enable': LaunchConfiguration('auto_enable'),
                    'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple'),
                    'gripper_exist': LaunchConfiguration('gripper_exist'),
                    'disable_gripper_auto_move': LaunchConfiguration('disable_gripper_auto_move'),
                    'rviz_ctrl_flag': LaunchConfiguration('rviz_ctrl_flag'),
                    'use_rosbridge': LaunchConfiguration('use_rosbridge'),
                    'operation_mode': LaunchConfiguration('operation_mode'),
                    'monitor_log_format': LaunchConfiguration('monitor_log_format'),
                    'monitor_rate_interval': LaunchConfiguration('monitor_rate_interval'),
                }],
                remappings=[('joint_states_single', 'joint_states')],
            )
            
            pub_guard_proc = ExecuteProcess(
                cmd=['python3', '-m', 'piper.single_publisher_guard_node'],
                name='single_pub_guard',
                output='screen'
            )
            
            return [loader_node, servo_node, piper_node, pub_guard_proc]

    core_nodes = OpaqueFunction(function=create_core_nodes)

    # Core nodes are now handled by the create_core_nodes function above

    # -----------------------
    # Mode-specific components
    # -----------------------
    
    # Replay logger (in replay and monitor modes)
    def create_replay_logger(context, *args, **kwargs):
        mode = context.launch_configurations.get('operation_mode', 'teleop')
        if mode in ['replay', 'monitor']:
            return [Node(
                package='piper',
                executable='piper_replay_logger',
                name='replay_logger',
                output='screen',
            )]
        return []

    replay_logger_entities = OpaqueFunction(function=create_replay_logger)

    # Gello entities (only in teleop mode)
    def create_gello_entities(context, *args, **kwargs):
        mode = context.launch_configurations.get('operation_mode', 'teleop')
        gello_exist = context.launch_configurations.get('gello_exist', 'false').lower() == 'true'
        
        if mode == 'teleop' and gello_exist:
            gello_entities = []
            
            # Determine if PiperGello sources are present
            for _gello_dir in ('/PiperGello', '/app/PiperGello'):
                launch_nodes_path = os.path.join(_gello_dir, 'experiments', 'launch_nodes.py')
                run_env_path = os.path.join(_gello_dir, 'experiments', 'run_env.py')
                if os.path.isfile(launch_nodes_path) and os.path.isfile(run_env_path):
                    # Build commands only when scripts exist
                    launch_nodes_cmd = (
                        f'PIPER_DIR={_gello_dir}; '
                        'exec python3 ${PIPER_DIR}/experiments/launch_nodes.py '
                        '--robot=piper --robot-ip=localhost'
                    )

                    run_env_cmd = (
                        f'PIPER_DIR={_gello_dir}; '
                        'exec python3 ${PIPER_DIR}/experiments/run_env.py --agent=gello '
                        '--gello_port=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NMKV-if00-port0'
                    )

                    gello_launch_nodes_proc = ExecuteProcess(
                        cmd=['bash', '-c', launch_nodes_cmd],
                        name='gello_launch_nodes',
                        output='screen',
                        log_cmd=True,
                        env={
                            'PYTHONUNBUFFERED': '1',
                            'PYTHONPATH': f'{_gello_dir}:${{PYTHONPATH}}',
                            'DISABLE_GRIPPER_AUTO_MOVE': 'true',
                        }
                    )

                    gello_run_env_proc = ExecuteProcess(
                        cmd=['bash', '-c', run_env_cmd],
                        name='gello_run_env',
                        output='screen',
                        log_cmd=True,
                        env={
                            'PYTHONUNBUFFERED': '1',
                            'PYTHONPATH': f'{_gello_dir}:${{PYTHONPATH}}',
                            'DISABLE_GRIPPER_AUTO_MOVE': 'true'
                        },
                    )

                    gello_entities.extend([gello_launch_nodes_proc, gello_run_env_proc])
                    break  # Found valid Gello dir, no need to search further
            
            # Add delay for Gello startup
            if gello_entities:
                return [TimerAction(
                    period=3.0,  # 3 second delay
                    actions=gello_entities
                )]
        
        return []

    gello_entities = OpaqueFunction(function=create_gello_entities)

    # -----------------------
    # Launch description
    # -----------------------
    return LaunchDescription([
        # Arguments
        operation_mode_arg,
        gello_exist_arg,
        can_port_arg,
        auto_enable_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        disable_gripper_auto_move_arg,
        rviz_ctrl_flag_arg,
        use_rosbridge_arg,
        gripper_config_arg,
        monitor_log_format_arg,
        monitor_rate_interval_arg,
        
        # Mode configuration (must run before nodes)
        mode_config,
        
        # Hardware processes and core nodes (conditional)
        hardware_processes,
        core_nodes,
        
        # Mode-specific entities
        replay_logger_entities,
        gello_entities,
    ])