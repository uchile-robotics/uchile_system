ARG BASE_OS=nvcr.io/nvidia/cuda:12.6.3-base-ubuntu24.04

FROM ${BASE_OS}

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get -y full-upgrade

# locale
RUN apt-get update && apt-get install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# base dependencies
RUN apt-get update && apt-get install -y git wget dbus-x11 lxqt-core tigervnc-standalone-server openssh-server vim curl python3-pip

# ROS2
RUN apt-get install -y software-properties-common
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get -y full-upgrade

RUN apt-get install -y ros-jazzy-desktop

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Gazebo
RUN apt-get update && apt-get -y full-upgrade
RUN apt-get install -y ros-jazzy-ros-gz

# PyTorch
RUN pip3 install torch torchvision torchaudio --break-system-packages

# authorize SSH connection with root account
RUN sed -i 's/#PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config

RUN echo "root:docker"|chpasswd

EXPOSE 22

ENTRYPOINT service ssh restart && bash
