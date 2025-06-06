from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start two Piper controllers – one for each CAN-bus connected arm.

    * Arm-1 is assumed to be on Linux CAN interface ``can0`` and will live in
      ROS namespace ``/arm1``.  All its topics/services will therefore appear
      under ``/arm1`` – e.g. ``/arm1/joint_ctrl_single``.
    * Arm-2 is on ``can1`` and is placed in namespace ``/arm2``.

    NOTE: Only arm1 has rosbridge enabled; both arms' data is accessible
    through the same websocket since they're in the same ROS domain.

    Adjust the ``parameters`` dictionaries below if you need to tweak any of
    the runtime parameters (auto-enable, gripper presence, …).
    """

    arm1_node = Node(
        package="piper",
        executable="piper_single_ctrl",
        namespace="arm1",
        name="piper_ctrl_arm1",
        parameters=[
            {"can_port": "can0"},
            {"auto_enable": True},       # automatically enable on startup
            {"use_rosbridge": True},     # only enable rosbridge on one arm!
            {"gripper_exist": True},     # change to False if no gripper
        ],
        output="screen",
    )

    arm2_node = Node(
        package="piper",
        executable="piper_single_ctrl",
        namespace="arm2",
        name="piper_ctrl_arm2",
        parameters=[
            {"can_port": "can1"},
            {"auto_enable": True},
            {"use_rosbridge": False},    # rosbridge already started by arm1
            {"gripper_exist": True},     # change to False if no gripper
        ],
        output="screen",
    )

    return LaunchDescription([arm1_node, arm2_node]) 