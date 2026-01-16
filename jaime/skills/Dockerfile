FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive

# Base deps
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
    ros-jazzy-joy-linux \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

RUN pip3 install transforms3d torch torchvision torchaudio opencv-python termcolor shapely --break-system-packages --ignore-installed

WORKDIR /home/jaime_ws/src
RUN git clone -b feat-jazzy https://github.com/uchile-robotics/uchile_common.git


WORKDIR /home/jaime_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# CycloneDDS config and basic shell setup
COPY config/cyclonedds.xml /
RUN echo "source /home/jaime_ws/install/setup.bash" >> /root/.bashrc && \
    echo "alias c='clear'" >> /root/.bashrc && \
    apt clean && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN pip install "setuptools<80" --break-system-packages
