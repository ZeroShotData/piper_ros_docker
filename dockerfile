# Dockerfile for AgileX Robotic Arm with ROS 2 Humble on AMD64

FROM ros:humble

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
    ros-humble-topic-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    supervisor \
    openssh-server \
    ethtool \
    can-utils \
    net-tools

# Install Python packages
RUN pip3 install python-can piper_sdk scipy

# Update rosdep (init is already done in base image)
RUN rosdep update

# Create ROS 2 workspace
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# Copy only the 'src' directory into the workspace
COPY src ./src

# Copy CAN configuration scripts to root
COPY can_activate.sh /root/
COPY can_config.sh /root/
COPY find_all_can_port.sh /root/
RUN chmod +x /root/*.sh

# Copy supervisord configuration
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Install package dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN colcon build

# Configure SSH on port 2222
RUN echo 'root:1234' | chpasswd && \
    sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config && \
    sed -i 's/Port 22/Port 2222/' /etc/ssh/sshd_config && \
    mkdir -p /var/run/sshd

# Expose ports
EXPOSE 2222 8765

# Configure Git and SSH for GitHub
ENV GIT_USER_NAME="JulienRineau"
ENV GIT_USER_EMAIL="julien.rineau@berkeley.edu"
RUN git config --global user.name "$GIT_USER_NAME" && \
    git config --global user.email "$GIT_USER_EMAIL" && \
    mkdir -p /root/.ssh && \
    ssh-keyscan github.com >> /root/.ssh/known_hosts

# Source the ROS 2 setup scripts and add useful aliases
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "alias activate_can='bash /root/can_activate.sh can0 1000000'" >> ~/.bashrc && \
    echo "alias start_piper='ros2 launch piper start_single_piper.launch.py'" >> ~/.bashrc && \
    echo "alias start_piper_rviz='ros2 launch piper start_single_piper_rviz.launch.py'" >> ~/.bashrc

CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf"]
