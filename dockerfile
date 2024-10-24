# Dockerfile for AgileX Robotic Arm with ROS 2 Humble on ARM64 including foxglove_bridge and additional dependencies

FROM ros:humble-ros-base-jammy

# Install necessary system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    build-essential \
    python3-colcon-common-extensions \
    ros-humble-foxglove-bridge \
    ros-humble-vision-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-geometry \
    ros-humble-topic-tools

# Install Python packages
RUN pip3 install python-can piper_sdk

# Initialize rosdep
RUN rosdep init && rosdep update

# Create ROS 2 workspace
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# Clone the AgileX Robotic Arm ROS 2 packages (update URLs as needed)
RUN git clone https://github.com/agilexrobotics/piper_ros2.git src/piper_ros2
RUN git clone https://github.com/agilexrobotics/piper_description_ros2.git src/piper_description_ros2
RUN git clone https://github.com/agilexrobotics/piper_msgs_ros2.git src/piper_msgs_ros2

# Install package dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN colcon build

# Source the ROS 2 setup scripts
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
