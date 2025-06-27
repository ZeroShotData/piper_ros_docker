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
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port used by the Piper controller.'
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Automatically enable the Piper controller.'
    )

    rviz_ctrl_flag_arg = DeclareLaunchArgument(
        'rviz_ctrl_flag',
        default_value='false',
        description='Launch rviz visualisation.'
    )

    gello_exist_arg = DeclareLaunchArgument(
        'gello_exist',
        default_value='true',
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
        default_value='true',
        description='Start rosbridge server for remote access.'
    )

    # Gripper YAML configuration (required, no default)
    gripper_config_arg = DeclareLaunchArgument(
        'gripper_config',
        description='Absolute path to gripper YAML configuration.'
    )

    # -----------------------
    # Nodes
    # -----------------------
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

    # Loader node – validates YAML and publishes parameters
    loader_node = Node(
        package='gripper_config_loader',
        executable='gripper_config_loader',
        name='gripper_config_loader',
        output='screen',
        parameters=[{'gripper_config': LaunchConfiguration('gripper_config'),
                     'gello_exist': LaunchConfiguration('gello_exist')}])

    servo_node = Node(
        package='st3215_driver',
        executable='st3215_servo',
        name='st3215_servo',
        output='screen',
        # Rely on parameters published by loader_node – no CLI defaults
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
        }],
        remappings=[('joint_states_single', 'joint_states')],
    )

    # -----------------------
    # Optional PiperGello helpers
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
                condition=IfCondition(LaunchConfiguration('gello_exist')),
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
                condition=IfCondition(LaunchConfiguration('gello_exist')),
                env={'PYTHONUNBUFFERED': '1', 'PYTHONPATH': f'{_gello_dir}:${{PYTHONPATH}}',
                     'DISABLE_GRIPPER_AUTO_MOVE': 'true'},
            )

            gello_entities.extend([gello_launch_nodes_proc, gello_run_env_proc])
            break  # Found valid Gello dir, no need to search further

    # Wrap Gello entities in TimerAction to delay startup until rosbridge is ready
    if gello_entities:
        delayed_gello_entities = [
            TimerAction(
                period=3.0,  # 3 second delay
                actions=gello_entities
            )
        ]
    else:
        delayed_gello_entities = []

    # -----------------------
    # Helper: keep gripper flag consistent with gello flag
    # -----------------------
    def sync_gripper_with_gello(context, *args, **kwargs):
        if context.launch_configurations.get('gello_exist', 'true').lower() == 'false':
            # Force gripper to be disabled when gello bridge is disabled
            context.launch_configurations['gripper_exist'] = 'false'
            # Also turn off rosbridge unless the user explicitly overrides later
            if context.launch_configurations.get('use_rosbridge', 'true').lower() != 'false':
                context.launch_configurations['use_rosbridge'] = 'false'
        return []

    sync_flags = OpaqueFunction(function=sync_gripper_with_gello)

    # -----------------------
    # Runtime safety guard – always included
    # -----------------------
    pub_guard_proc = ExecuteProcess(
        cmd=['python3', '-m', 'piper.single_publisher_guard_node'],
        name='single_pub_guard',
        output='screen'
    )

    # -----------------------
    # Launch description
    # -----------------------
    return LaunchDescription([
        # arguments
        gello_exist_arg,
        can_port_arg,
        auto_enable_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        disable_gripper_auto_move_arg,
        rviz_ctrl_flag_arg,
        use_rosbridge_arg,
        gripper_config_arg,
        # flag sync helper (must run before nodes)
        sync_flags,
        # nodes
        can_activate_proc,
        create_serial_link,
        loader_node,
        servo_node,
        piper_node,
        pub_guard_proc,
        # Optional helpers (only added if scripts exist)
        *delayed_gello_entities,
    ]) 