from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition


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

    no_servo_arg = DeclareLaunchArgument(
        'no_servo',
        default_value='false',
        description='Set to true to skip launching the servo driver.'
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

    servo_node = Node(
        package='st3215_driver',
        executable='st3215_servo',
        name='st3215_servo',
        output='screen',
        parameters=[{'device': LaunchConfiguration('device')}],
        condition=UnlessCondition(LaunchConfiguration('no_servo'))
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
        remappings=[
            ('joint_ctrl_single', '/joint_states'),
        ]
    )

    # Helper bash command to activate / create venv and run python script
    launch_nodes_cmd = (
        'if [ ! -f /PiperGello/.venv/bin/activate ]; then '
        'python3 -m venv /PiperGello/.venv && '
        'source /PiperGello/.venv/bin/activate && '
        'pip install -q -r /PiperGello/requirements.txt; '
        'else source /PiperGello/.venv/bin/activate; fi && '
        'exec python /PiperGello/experiments/launch_nodes.py '
        '--robot=piper --robot-ip=localhost '
        '--servo_host_port_pair=10.0.207.135:9876:9877'
    )

    run_env_cmd = (
        'if [ ! -f /PiperGello/.venv/bin/activate ]; then '
        'python3 -m venv /PiperGello/.venv && '
        'source /PiperGello/.venv/bin/activate && '
        'pip install -q -r /PiperGello/requirements.txt; '
        'else source /PiperGello/.venv/bin/activate; fi && '
        'exec python /PiperGello/experiments/run_env.py '
        '--agent=gello '
        '--gello_port=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NMKV-if00-port0'
    )

    gello_launch_nodes_proc = ExecuteProcess(
        cmd=['bash', '-c', launch_nodes_cmd],
        name='gello_launch_nodes',
        output='screen'
    )

    gello_run_env_proc = ExecuteProcess(
        cmd=['bash', '-c', run_env_cmd],
        name='gello_run_env',
        output='screen'
    )

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
        no_servo_arg,
        # nodes
        can_activate_proc,
        servo_node,
        piper_node,
        gello_launch_nodes_proc,
        gello_run_env_proc,
    ]) 