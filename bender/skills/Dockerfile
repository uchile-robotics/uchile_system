FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive

# Base deps
RUN apt-get update && apt-get install -y \
    git wget curl vim python3-pip python3-colcon-common-extensions \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Locale
RUN apt-get update && apt-get install -y locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# Gazebo (ros_gz)
RUN apt-get update && apt-get install -y ros-jazzy-ros-gz && rm -rf /var/lib/apt/lists/*

# PyTorch + OpenCV (con GPU)
RUN pip3 install torch torchvision torchaudio opencv-python --break-system-packages --ignore-installed
COPY config/cyclonedds.xml /
# Entrypoint ROS
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
ENTRYPOINT ["/bin/bash"]
