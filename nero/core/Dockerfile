# =============================================================================
#  UChile System - AgileX NERO 7-DoF arm (ROS 2 Jazzy)
#
#  Driver : https://github.com/agilexrobotics/agx_arm_ros   (branch ros2)
#  SDK    : https://github.com/agilexrobotics/pyAgxArm       (branch master)
#
#  The arm talks over SocketCAN (USB-CAN module -> `can_nero` network iface).
#  The CAN interface is created/activated on the HOST (udev rule, see
#  config/udev_scripts) and is visible in the container via network_mode: host.
# =============================================================================
# desktop image -> includes rviz2 (needed for MoveIt / visualization launches)
ARG BASE_OS=osrf/ros:jazzy-desktop
FROM ${BASE_OS}

ARG DEBIAN_FRONTEND=noninteractive

# Upstream refs (branch, tag or commit SHA). Pin them for reproducible deploys:
#   docker compose build --build-arg AGX_ARM_ROS_REF=<sha> --build-arg PYAGXARM_REF=<sha>
ARG AGX_ARM_ROS_REF=ros2
ARG PYAGXARM_REF=master

# ---------------------------------------------------------------------------
# Locale (MoveIt requires an English LC_NUMERIC, otherwise it fails parsing)
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends locales && \
    locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    LC_NUMERIC=en_US.UTF-8

# ---------------------------------------------------------------------------
# System + ROS dependencies
# (equivalent to upstream scripts/agx_arm_install_deps.sh, minus the
#  `ros-jazzy-moveit*` / `ros-jazzy-control*` globs, which pull GBs of junk)
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get full-upgrade -y && \
    apt-get install -y --no-install-recommends \
        git wget curl vim nano openssh-server \
        python3-pip python3-numpy python3-scipy \
        can-utils ethtool iproute2 kmod usbutils \
        ros-dev-tools \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-controller-manager \
        ros-jazzy-joint-trajectory-controller \
        ros-jazzy-joint-state-broadcaster \
        ros-jazzy-gripper-controllers \
        ros-jazzy-trajectory-msgs \
        ros-jazzy-topic-tools \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-xacro \
        ros-jazzy-moveit \
        ros-jazzy-moveit-planners \
        ros-jazzy-moveit-ros-visualization \
        ros-jazzy-moveit-ros-warehouse \
        ros-jazzy-moveit-ros-perception \
        ros-jazzy-moveit-simple-controller-manager \
        ros-jazzy-moveit-kinematics \
        ros-jazzy-moveit-configs-utils \
        ros-jazzy-moveit-ros-move-group \
        ros-jazzy-moveit-setup-assistant \
        ros-jazzy-tf-transformations \
        x11-apps libxkbcommon-x11-0 libxcb-xinerama0 \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Python SDK (pyAgxArm) + python deps used by agx_arm_ctrl
# ---------------------------------------------------------------------------
RUN pip3 install --no-cache-dir --break-system-packages python-can && \
    git clone https://github.com/agilexrobotics/pyAgxArm.git /opt/pyAgxArm && \
    cd /opt/pyAgxArm && git checkout ${PYAGXARM_REF} && \
    pip3 install --no-cache-dir --break-system-packages . && \
    python3 -c "import pyAgxArm, can; print('pyAgxArm', pyAgxArm.__file__)"

# ---------------------------------------------------------------------------
# ROS 2 workspace with agx_arm_ros (+ agx_arm_urdf submodule)
# ---------------------------------------------------------------------------
WORKDIR /home/nero_ws/src
RUN git clone -b ros2 https://github.com/agilexrobotics/agx_arm_ros.git && \
    cd agx_arm_ros && git checkout ${AGX_ARM_ROS_REF} && \
    git submodule update --init --recursive

# warehouse_ros_mongo is not released for Jazzy and is only used by the
# optional warehouse DB launch -> skip it.
WORKDIR /home/nero_ws
RUN apt-get update && \
    (rosdep init || true) && rosdep update --rosdistro=jazzy && \
    rosdep install -i --from-paths src --rosdistro=jazzy -y -r \
        --skip-keys="warehouse_ros_mongo" && \
    rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# ---------------------------------------------------------------------------
# Runtime config
# ---------------------------------------------------------------------------
COPY config/cyclonedds.xml /
COPY config/nero_aliases.sh /etc/profile.d/nero_aliases.sh

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /home/nero_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /etc/profile.d/nero_aliases.sh" >> /root/.bashrc && \
    echo "alias c='clear'" >> /root/.bashrc

RUN sed -i 's/#PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    echo "root:docker" | chpasswd

EXPOSE 22
ENTRYPOINT ["bash", "-c", "service ssh restart && exec bash"]
