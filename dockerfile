# Dockerfile for AgileX Robotic Arm with ROS 2 Humble on ARM64 including foxglove_bridge and additional dependencies

FROM arm64v8/ros:humble-ros-base-jammy

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
    supervisor \
    openssh-server

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

# Configure SSH on port 2222
RUN echo 'root:1234' | chpasswd && \
    sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config && \
    sed -i 's/Port 22/Port 2222/' /etc/ssh/sshd_config && \
    mkdir -p /var/run/sshd

# Expose SSH port
EXPOSE 2222

# Copy supervisord configuration
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Configure Git and SSH for GitHub
ENV GIT_USER_NAME="JulienRineau"
ENV GIT_USER_EMAIL="julien.rineau@berkeley.edu"
RUN git config --global user.name "$GIT_USER_NAME" && \
    git config --global user.email "$GIT_USER_EMAIL" && \
    mkdir -p /root/.ssh && \
    ssh-keyscan github.com >> /root/.ssh/known_hosts

# Source the ROS 2 setup scripts
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf"]
