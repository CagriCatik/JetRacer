FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy
ENV WORKSPACE=/workspaces/JetRacer-ROS2

SHELL ["/bin/bash", "-c"]

WORKDIR ${WORKSPACE}

# --------------------------------------------------------------------
# Base system dependencies
# --------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    software-properties-common \
    build-essential \
    cmake \
    git \
    wget \
    nano \
    vim \
    udev \
    i2c-tools \
    v4l-utils \
    portaudio19-dev \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-venv \
    python3-opencv \
    python3-pyaudio \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# --------------------------------------------------------------------
# Add ROS 2 Foxy apt repository
# --------------------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# --------------------------------------------------------------------
# ROS 2 Foxy core packages
# --------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-rplidar-ros \
    ros-${ROS_DISTRO}-gscam \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------------------
# Optional packages that may or may not exist in Foxy apt repos
# --------------------------------------------------------------------
RUN apt-get update && \
    for pkg in \
        ros-${ROS_DISTRO}-foxglove-bridge \
    ; do \
        if apt-cache show "$pkg" >/dev/null 2>&1; then \
            echo "Installing optional package: $pkg"; \
            apt-get install -y --no-install-recommends "$pkg"; \
        else \
            echo "Skipping unavailable optional package: $pkg"; \
        fi; \
    done && \
    rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------------------
# Python dependencies
# --------------------------------------------------------------------
COPY requirements.txt /tmp/requirements.txt

RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel && \
    if [ -s /tmp/requirements.txt ]; then \
        python3 -m pip install --no-cache-dir -r /tmp/requirements.txt; \
    else \
        echo "No Python requirements to install"; \
    fi

# --------------------------------------------------------------------
# Initialize rosdep
# --------------------------------------------------------------------
RUN rosdep init 2>/dev/null || true && \
    rosdep update --rosdistro ${ROS_DISTRO} || true

# --------------------------------------------------------------------
# Workspace setup
# --------------------------------------------------------------------
RUN mkdir -p ${WORKSPACE}/src

# --------------------------------------------------------------------
# Shell setup
# --------------------------------------------------------------------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /root/.bashrc && \
    echo "alias build_workspace='cd ${WORKSPACE} && colcon build --symlink-install'" >> /root/.bashrc && \
    echo "cd ${WORKSPACE}" >> /root/.bashrc

CMD ["/bin/bash"]