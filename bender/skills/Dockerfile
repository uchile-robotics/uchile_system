FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive

# Base dependencies
RUN apt-get update && apt-get install -y \
    git wget curl vim python3-pip \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-ros-gz \
    ros-jazzy-navigation2 \
    ros-jazzy-cartographer-ros \
    ros-jazzy-tf-transformations \
    ros-dev-tools \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-joy \
    && rm -rf /var/lib/apt/lists/*

# Fix for setuptools --editable issue (PEP 660 support)
RUN pip install -U pip setuptools wheel --break-system-packages --ignore-installed
ENV PYTHONPATH=/usr/local/lib/python3.12/dist-packages:$PYTHONPATH

# Locale configuration
RUN apt-get update && apt-get install -y locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# PyTorch + OpenCV (with GPU)
RUN pip3 install transforms3d torch torchvision torchaudio opencv-python termcolor --break-system-packages --ignore-installed

# CycloneDDS config and basic shell setup
COPY config/cyclonedds.xml /
RUN echo "source /home/bender_ws/install/setup.bash" >> /root/.bashrc && \
    echo "alias c='clear'" >> /root/.bashrc && \
    apt clean && rm -rf /var/lib/apt/lists/*

# Default shell and ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
