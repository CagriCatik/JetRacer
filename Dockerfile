FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

# Prevent interactive prompts during apt installations
ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE=/workspaces/JetRacer-ROS2
WORKDIR $WORKSPACE

# Install ROS 2 Humble runtime dependencies and build tools needed by this workspace.
# This image is ROS 2 only (no ROS 1/ROS 2 bridge packages).
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-dev \
    python3-opencv \
    build-essential \
    i2c-tools \
    v4l-utils \
    portaudio19-dev \
    python3-pyaudio \
    ros-humble-foxglove-bridge \
    ros-humble-explore-lite \
    ros-humble-twist-mux \
    ros-humble-robot-localization \
    ros-humble-rplidar-ros \
    ros-humble-gscam \
    ros-humble-camera-info-manager \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-joy \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    ros-humble-cartographer-ros \
    ros-humble-nav2-bringup \
    ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep metadata (safe if already initialized in the base image)
RUN rosdep init 2>/dev/null || true

# Install Python runtime dependencies used by the ROS 2 nodes
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Add a convenient source alias to bashrc so users don't have to manually source ROS every time
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> ~/.bashrc \
    && echo "alias build_workspace='cd ${WORKSPACE} && colcon build --symlink-install'" >> ~/.bashrc \
    && echo "cd ${WORKSPACE}" >> ~/.bashrc

# Ensure the container keeps running for interactive access
CMD ["/bin/bash"]
