from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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

    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='false',
        description='Whether a gripper is attached.'
    )

    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='1',
        description='Scalar applied to gripper values.'
    )

    use_rosbridge_arg = DeclareLaunchArgument(
        'use_rosbridge',
        default_value='true',
        description='Start rosbridge server for remote access.'
    )

    # Servo specific args
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyACM0',
        description='Serial device for the ST3215 servo driver.'
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

    servo_node = Node(
        package='st3215_driver',
        executable='st3215_servo',
        name='st3215_servo',
        output='screen',
        parameters=[{'device': LaunchConfiguration('device')}],
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
            'rviz_ctrl_flag': LaunchConfiguration('rviz_ctrl_flag'),
            'use_rosbridge': LaunchConfiguration('use_rosbridge'),
        }],
        remappings=[('joint_states_single', 'joint_ctrl_single')],
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
                env={
                    'PYTHONUNBUFFERED': '1',
                    'PYTHONPATH': f'{_gello_dir}:${{PYTHONPATH}}',
                }
            )

            gello_run_env_proc = ExecuteProcess(
                cmd=['bash', '-c', run_env_cmd],
                name='gello_run_env',
                output='screen',
                log_cmd=True,
                env={'PYTHONUNBUFFERED': '1', 'PYTHONPATH': f'{_gello_dir}:${{PYTHONPATH}}'}
            )

            gello_entities.extend([gello_launch_nodes_proc, gello_run_env_proc])
            break  # Found valid Gello dir, no need to search further

    # -----------------------
    # Launch description
    # -----------------------
    return LaunchDescription([
        # arguments
        can_port_arg,
        auto_enable_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        rviz_ctrl_flag_arg,
        use_rosbridge_arg,
        device_arg,
        # nodes
        can_activate_proc,
        create_serial_link,
        servo_node,
        piper_node,
        # Optional helpers (only added if scripts exist)
        *gello_entities,
    ]) 