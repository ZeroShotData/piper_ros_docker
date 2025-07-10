# Dockerfile for AgileX Robotic Arm with ROS 2 Humble on AMD64 - Background Agent Version

# Use ROS 2 Humble base image that includes development tools
FROM ros:humble-ros-base

# Install necessary system dependencies and development tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    git \
    build-essential \
    python3-colcon-common-extensions \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-core \
    ros-humble-ament-package \
    ros-humble-ament-cmake-ros \
    ros-humble-ament-cmake-export-dependencies \
    ros-humble-ament-cmake-libraries \
    ros-humble-ament-cmake-target-dependencies \
    ros-humble-foxglove-bridge \
    ros-humble-vision-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-geometry \
    ros-humble-topic-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-moveit \
    ros-humble-rosbridge-server \
    supervisor \
    openssh-server \
    ethtool \
    can-utils \
    iproute2 \
    net-tools \
    python3-serial \
    sudo \
    curl \
    wget \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages needed for development
RUN pip3 install --no-cache-dir \
    python-can \
    piper_sdk \
    scipy \
    tyro \
    pandas \
    pyzmq \
    roslibpy \
    pyquaternion \
    dynamixel_sdk \
    pyarrow \
    fastparquet

# Create a non-root user for development
RUN useradd -m -s /bin/bash ubuntu && \
    echo 'ubuntu:ubuntu' | chpasswd && \
    usermod -aG sudo ubuntu && \
    echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to the ubuntu user
USER ubuntu

# Set working directory to user's home
WORKDIR /home/ubuntu

# Configure Git for the user
RUN git config --global user.name "JulienRineau" && \
    git config --global user.email "julien.rineau@berkeley.edu" && \
    mkdir -p /home/ubuntu/.ssh && \
    ssh-keyscan github.com >> /home/ubuntu/.ssh/known_hosts

# Set up ROS 2 environment for the user
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/ubuntu/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/humble/" >> /home/ubuntu/.bashrc && \
    echo "# Workspace setup will be added after cloning" >> /home/ubuntu/.bashrc && \
    echo "if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi" >> /home/ubuntu/.bashrc

# Add useful aliases for development
RUN echo "alias activate_can='bash ~/piper_ros_docker/can_activate.sh can0 1000000'" >> /home/ubuntu/.bashrc && \
    echo "alias start_piper='ros2 launch piper start_single_piper.launch.py'" >> /home/ubuntu/.bashrc && \
    echo "alias start_piper_rviz='ros2 launch piper start_single_piper_rviz.launch.py'" >> /home/ubuntu/.bashrc && \
    echo "alias start_piper_rosbridge='ros2 launch piper start_single_piper_rosbridge.launch.py'" >> /home/ubuntu/.bashrc && \
    echo "alias build_ws='cd ~/ros2_ws && colcon build --symlink-install'" >> /home/ubuntu/.bashrc

# Update rosdep database
RUN sudo rosdep update

# Expose commonly used ports for ROS development
EXPOSE 8765 9090 11311

# Default command - start bash shell
CMD ["/bin/bash"]
