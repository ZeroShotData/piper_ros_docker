"""DEPRECATED: This launch file is deprecated. Use piper_unified.launch.py instead.

Backward compatibility wrapper for websocket_logger.launch.py
This file provided rosbridge server + replay logger. Now integrated into unified launcher.

Maps to: ros2 launch piper piper_unified.launch.py operation_mode:=replay

Usage:
  ros2 launch piper websocket_logger.launch.py
  
Recommended:
  ros2 launch piper piper_unified.launch.py operation_mode:=replay gripper_config:=/app/configs/my_setup.yaml

Note: The unified launcher automatically starts rosbridge server and replay logger in replay mode.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Print deprecation warning
    import sys
    print("\n" + "="*60, file=sys.stderr)
    print("WARNING: websocket_logger.launch.py is DEPRECATED", file=sys.stderr)
    print("Please use: ros2 launch piper piper_unified.launch.py operation_mode:=replay", file=sys.stderr)
    print("Note: Rosbridge server and replay logger are automatically started in replay mode.", file=sys.stderr)
    print("="*60 + "\n", file=sys.stderr)
    
    # Gripper config argument with default
    gripper_config_arg = DeclareLaunchArgument(
        'gripper_config',
        default_value='/app/configs/my_setup.yaml',
        description='Absolute path to gripper YAML configuration.'
    )
    
    # Path to the unified launch file
    unified_launch_path = os.path.join(os.path.dirname(__file__), 'piper_unified.launch.py')
    
    # Include unified launch with replay mode (which includes rosbridge + replay logger)
    unified_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(unified_launch_path),
        launch_arguments={
            'operation_mode': 'replay',
            'gripper_config': LaunchConfiguration('gripper_config'),
        }.items(),
    )
    
    return LaunchDescription([gripper_config_arg, unified_launch])