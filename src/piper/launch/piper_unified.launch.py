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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
import os


def get_mode_defaults(mode, overrides=None):
    """Get mode-specific default values for parameters.
    
    Args:
        mode: Operation mode ('teleop', 'replay', or 'monitor')
        overrides: Dict of user-provided values to preserve
        
    Returns:
        Dict with final parameter values
    """
    if overrides is None:
        overrides = {}
    
    # Mode-specific defaults
    defaults = {
        'teleop': {
            'auto_enable': 'true',
            'gello_exist': 'true', 
            'use_rosbridge': 'true',
            'gripper_exist': 'true'
        },
        'replay': {
            'auto_enable': 'false',
            'gello_exist': 'false',
            'use_rosbridge': 'true', 
            'gripper_exist': 'true'
        },
        'monitor': {
            'auto_enable': 'false',
            'gello_exist': 'false',
            'use_rosbridge': 'true',
            'gripper_exist': 'false'  # Monitor mode: no gripper by default
        }
    }
    
    result = defaults.get(mode, defaults['teleop']).copy()
    
    # Apply user overrides
    for key, value in overrides.items():
        if value:  # Only override if user provided a non-empty value
            result[key] = value
    
    # Sync gripper with gello logic (from original start_piper.launch.py)
    if result['gello_exist'].lower() == 'false':
        result['gripper_exist'] = 'false'
        if result['use_rosbridge'].lower() != 'false':
            result['use_rosbridge'] = 'false'
    
    return result


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
        default_value=PythonExpression([
            "'true' if '", LaunchConfiguration('operation_mode'), "' == 'teleop' else 'false'"
        ]),
        description='Automatically enable the Piper controller.'
    )

    rviz_ctrl_flag_arg = DeclareLaunchArgument(
        'rviz_ctrl_flag',
        default_value='false',
        description='Launch rviz visualisation.'
    )

    gello_exist_arg = DeclareLaunchArgument(
        'gello_exist',
        default_value=PythonExpression([
            "'true' if '", LaunchConfiguration('operation_mode'), "' == 'teleop' else 'false'"
        ]),
        description='Whether the Gello bridge is running.'
    )

    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value=PythonExpression([
            "'false' if ('", LaunchConfiguration('operation_mode'), "' == 'monitor' or '", LaunchConfiguration('gello_exist'), "' == 'false') else 'true'"
        ]),
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
        default_value=PythonExpression([
            "'false' if '", LaunchConfiguration('gello_exist'), "' == 'false' else 'true'"
        ]),
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
    # Hardware Setup (inline, not in monitor mode)
    # -----------------------
    # Run CAN configuration in all modes except pure monitor
    can_activate_proc = ExecuteProcess(
        cmd=['bash', '/app/can_activate.sh', 'can0', '1000000', '1-1.1:1.0'],
        name='activate_can',
        output='screen',
        condition=UnlessCondition(
            PythonExpression([
                "'", LaunchConfiguration('operation_mode'), "' == 'monitor'"
            ])
        )
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
        ],
        condition=UnlessCondition(
            PythonExpression([
                "'", LaunchConfiguration('operation_mode'), "' == 'monitor'"
            ])
        )
    )

    # -----------------------
    # Core Nodes (deterministic order)
    # -----------------------
    # Loader node – validates YAML and publishes parameters (ALWAYS FIRST)
    loader_node = Node(
        package='gripper_config_loader',
        executable='gripper_config_loader',
        name='gripper_config_loader',
        output='screen',
        parameters=[{
            'gripper_config': LaunchConfiguration('gripper_config'),
            'gello_exist': LaunchConfiguration('gello_exist')
        }],
        condition=UnlessCondition(
            PythonExpression([
                "'", LaunchConfiguration('operation_mode'), "' == 'monitor'"
            ])
        )
    )

    # Servo node (conditional on gripper existence)
    servo_node = Node(
        package='st3215_driver',
        executable='st3215_servo',
        name='st3215_servo',
        output='screen',
        parameters=[],  # Relies on parameters published by loader_node
        condition=IfCondition(LaunchConfiguration('gripper_exist'))
    )

    # Piper controller node (unified parameters for all modes)
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

    # Runtime safety guard (always included except monitor mode)
    pub_guard_proc = ExecuteProcess(
        cmd=['python3', '-m', 'piper.single_publisher_guard_node'],
        name='single_pub_guard',
        output='screen',
        condition=UnlessCondition(
            PythonExpression([
                "'", LaunchConfiguration('operation_mode'), "' == 'monitor'"
            ])
        )
    )

    # -----------------------
    # Conditional Components
    # -----------------------
    # Replay logger (in replay and monitor modes)
    replay_logger_node = Node(
        package='piper',
        executable='piper_replay_logger',
        name='replay_logger',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('operation_mode'), "' in ['replay', 'monitor']"
            ])
        )
    )

    # -----------------------
    # Gello Components (teleop mode only)
    # -----------------------
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
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration('operation_mode'), "' == 'teleop' and '", LaunchConfiguration('gello_exist'), "' == 'true'"
                    ])
                ),
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
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration('operation_mode'), "' == 'teleop' and '", LaunchConfiguration('gello_exist'), "' == 'true'"
                    ])
                ),
                env={
                    'PYTHONUNBUFFERED': '1', 
                    'PYTHONPATH': f'{_gello_dir}:${{PYTHONPATH}}',
                    'DISABLE_GRIPPER_AUTO_MOVE': 'true'
                },
            )

            gello_entities.extend([gello_launch_nodes_proc, gello_run_env_proc])
            break  # Found valid Gello dir, no need to search further

    # Wrap Gello entities in TimerAction to delay startup until rosbridge is ready
    delayed_gello_entities = []
    if gello_entities:
        delayed_gello_entities = [
            TimerAction(
                period=3.0,  # 3 second delay
                actions=gello_entities
            )
        ]

    # -----------------------
    # Launch Description (deterministic order)
    # -----------------------
    return LaunchDescription([
        # (a) Declare all arguments
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
        
        # (b) Hardware processes (conditional)
        can_activate_proc,
        create_serial_link,
        
        # (c) Core nodes in guaranteed order
        loader_node,          # ALWAYS FIRST - publishes gripper parameters
        servo_node,           # Depends on loader parameters
        piper_node,           # Main controller
        pub_guard_proc,       # Safety guard
        
        # (d) Conditional components
        replay_logger_node,   # Replay/monitor modes only
        
        # (e) Gello helpers (teleop mode only, delayed startup)
        *delayed_gello_entities,
    ])