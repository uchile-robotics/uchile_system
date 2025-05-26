ARG BASE_OS=ubuntu:24.04

FROM ${BASE_OS}

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get -y full-upgrade

# locale

RUN apt-get update && apt-get install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# base dependencies
RUN apt update && apt upgrade -y
RUN apt-get install -y git wget dbus-x11 lxqt-core tigervnc-standalone-server openssh-server vim curl python3-pip nano

# ROS2


RUN apt-get install -y software-properties-common
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get -y full-upgrade
RUN apt update --fix-missing

RUN apt-get install -y ros-jazzy-ros-base

RUN apt-get update && apt-get install -y \
    python3-flake8-blind-except \
    python3-flake8-class-newline \
    python3-flake8-deprecated \
    python3-mypy \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-mock \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-pytest-runner \
    python3-pytest-timeout \
    doxygen \
    ros-dev-tools \
    libboost-dev \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-twist-mux \
    ros-jazzy-cartographer-ros
    


WORKDIR /home
RUN mkdir -p base_controller/src
WORKDIR /home/base_controller/src
RUN git clone https://github.com/reedhedges/AriaCoda.git && \ 
    git clone https://github.com/ruipaulorocha/rosaria2.git && \
    git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git
WORKDIR /home/base_controller/src/AriaCoda
RUN make -j 
RUN make install

WORKDIR /home/base_controller/src
RUN rm -r AriaCoda

RUN rosdep init && \
    rosdep fix-permissions && \
    rosdep update


RUN rosdep install -i --from-paths urg_node2 rosaria2 --rosdistro=jazzy -y

RUN sed -i 's/params_ether.yaml/params_serial.yaml/' urg_node2/launch/urg_node2.launch.py
WORKDIR /home/base_controller
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
RUN echo "source /opt/ros/jazzy/setup.bash\nsource /home/base_controller/install/setup.bash" >> /root/.bashrc
RUN echo "alias c='clear'" >> /root/.bashrc

# Config files for cartographer
COPY config/cyclonedds.xml /
COPY config/cartographer_config.lua /opt/ros/jazzy/share/cartographer/configuration_files 
# authorize SSH connection with root account
RUN sed -i 's/#PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config

RUN echo "root:docker"|chpasswd

EXPOSE 22

ENTRYPOINT ["bash", "-c", "service ssh restart && exec bash"]
