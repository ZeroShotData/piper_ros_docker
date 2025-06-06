#!/bin/bash
# start_dual_arms.sh - Script to start two Piper arm controllers
# Usage: ./start_dual_arms.sh [arm1_canbus] [arm2_canbus] [arm1_name] [arm2_name]
#   defaults: can0, can1, choupette, baguette

ARM1_CAN=${1:-can0}
ARM2_CAN=${2:-can1}
ARM1_NAME=${3:-choupette}
ARM2_NAME=${4:-baguette}

echo "Starting Piper controllers with:"
echo "  Left arm ($ARM1_NAME) on $ARM1_CAN"
echo "  Right arm ($ARM2_NAME) on $ARM2_CAN"

# Start the left arm first (with rosbridge enabled)
ros2 run piper piper_single_ctrl \
  --ros-args \
    -p can_port:=$ARM1_CAN \
    -p auto_enable:=true \
    -p gripper_exist:=true \
    -p rviz_ctrl_flag:=false \
    -p use_rosbridge:=true \
    -r __ns:=/$ARM1_NAME \
    -r __node:=${ARM1_NAME}_ctrl &

LEFT_PID=$!
echo "Left arm controller started (PID: $LEFT_PID)"

# Small delay to let rosbridge start properly
sleep 2

# Start the right arm (without redundant rosbridge)
ros2 run piper piper_single_ctrl \
  --ros-args \
    -p can_port:=$ARM2_CAN \
    -p auto_enable:=true \
    -p gripper_exist:=true \
    -p rviz_ctrl_flag:=false \
    -p use_rosbridge:=false \
    -r __ns:=/$ARM2_NAME \
    -r __node:=${ARM2_NAME}_ctrl &

RIGHT_PID=$!
echo "Right arm controller started (PID: $RIGHT_PID)"

echo "Both controllers running. Press CTRL+C to stop."
echo "  - Left arm topics are available at /$ARM1_NAME/..."
echo "  - Right arm topics are available at /$ARM2_NAME/..."
echo "  - All topics are accessible via rosbridge websocket at ws://localhost:9090"

# Wait for both processes and handle interrupts
wait_for_processes() {
  # Wait for the controllers to exit on their own or until interrupted
  wait $LEFT_PID $RIGHT_PID
}

# Handle cleanup on script termination
cleanup() {
  echo "Shutting down arm controllers..."
  kill $LEFT_PID $RIGHT_PID 2>/dev/null
  exit 0
}

# Set trap for SIGINT (Ctrl+C)
trap cleanup SIGINT SIGTERM

# Wait for processes
wait_for_processes 