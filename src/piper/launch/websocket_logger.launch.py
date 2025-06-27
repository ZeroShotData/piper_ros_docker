"""Launch rosbridge websocket server and replay test logger.

Input: Websocket messages via rosbridge on default port (9090)
Output: Console logs showing message count, rate, and joint positions from /joint_ctrl_single topic

Usage: ros2 launch piper websocket_logger.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Start rosbridge websocket server
    rosbridge_proc = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        name='rosbridge_websocket',
        output='screen'
    )

    # Start the replay test logger
    replay_logger_node = Node(
        package='piper',
        executable='piper_replay_logger',
        name='replay_logger',
        output='screen',
    )

    return LaunchDescription([
        rosbridge_proc,
        replay_logger_node,
    ])