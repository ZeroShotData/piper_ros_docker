"""DEPRECATED: This launch file is deprecated. Use piper_unified.launch.py instead.

Backward compatibility wrapper for start_single_piper_rosbridge.launch.py
Maps to: ros2 launch piper piper_unified.launch.py operation_mode:=replay use_rosbridge:=true

Usage:
  ros2 launch piper start_single_piper_rosbridge.launch.py
  
Recommended:
  ros2 launch piper piper_unified.launch.py operation_mode:=replay use_rosbridge:=true

This file forced use_rosbridge=true which is now the default for replay mode.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Print deprecation warning
    import sys
    print("\n" + "="*70, file=sys.stderr)
    print("WARNING: start_single_piper_rosbridge.launch.py is DEPRECATED", file=sys.stderr)
    print("Please use: ros2 launch piper piper_unified.launch.py operation_mode:=replay use_rosbridge:=true", file=sys.stderr)
    print("="*70 + "\n", file=sys.stderr)
    
    # Preserve all original arguments for backward compatibility
    args = [
        DeclareLaunchArgument('can_port', default_value='can0', description='CAN port to be used by the Piper node.'),
        DeclareLaunchArgument('auto_enable', default_value='true', description='Automatically enable the Piper node.'),
        DeclareLaunchArgument('rviz_ctrl_flag', default_value='false', description='Start rviz flag.'),
        DeclareLaunchArgument('gripper_exist', default_value='true', description='gripper'),
        DeclareLaunchArgument('gripper_val_mutiple', default_value='1', description='gripper'),
        DeclareLaunchArgument('use_rosbridge', default_value='true', description='Start rosbridge server for remote access'),
    ]
    
    # Path to the unified launch file
    unified_launch_path = os.path.join(os.path.dirname(__file__), 'piper_unified.launch.py')
    
    # Include unified launch with replay mode and rosbridge enabled
    unified_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(unified_launch_path),
        launch_arguments={
            'operation_mode': 'replay',
            'can_port': LaunchConfiguration('can_port'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'rviz_ctrl_flag': LaunchConfiguration('rviz_ctrl_flag'),
            'gripper_exist': LaunchConfiguration('gripper_exist'),
            'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple'),
            'use_rosbridge': LaunchConfiguration('use_rosbridge'),
            'gripper_config': '/app/configs/my_setup.yaml',  # Default config
        }.items(),
    )
    
    return LaunchDescription([*args, unified_launch])