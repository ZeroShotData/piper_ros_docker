from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Path to the full-feature Piper launcher that runs the parameter loader
    piper_launch_path = os.path.join(
        os.path.dirname(__file__),
        'start_piper.launch.py'
    )

    # Default gripper config path (can be overridden via CLI argument)
    default_cfg = '/app/configs/my_setup.yaml'

    # Include existing launch with replay-friendly arguments
    piper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(piper_launch_path),
        launch_arguments={
            'use_rosbridge': 'true',   # Start rosbridge automatically
            'gello_exist': 'false',    # Disable Gello bridge
            'auto_enable': 'false',    # Wait for enable flag from host
            'gripper_config': default_cfg,
        }.items(),
    )

    # Replay logger node – prints incoming JointState rate
    replay_logger_node = Node(
        package='piper',
        executable='piper_replay_logger',
        name='replay_logger',
        output='screen',
    )

    return LaunchDescription([
        piper_launch,
        replay_logger_node,
    ]) 