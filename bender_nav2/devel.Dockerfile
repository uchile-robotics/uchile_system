ARG BASE_OS=ubuntu:24.04
FROM ${BASE_OS}

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
RUN apt-get update && apt-get full-upgrade -y && \
    apt-get install -y --no-install-recommends \
        locales \
        git wget openssh-server vim curl nano python3-pip \
        software-properties-common \
        doxygen \
        libboost-dev && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 

RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-twist-mux \
        ros-jazzy-cartographer-ros \
        ros-jazzy-tf-transformations \
        ros-dev-tools

WORKDIR /home/base_controller/src
RUN git clone https://github.com/reedhedges/AriaCoda.git && \
    git clone https://github.com/ruipaulorocha/rosaria2.git && \
    git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git && \
    git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

WORKDIR /home/base_controller/src/AriaCoda
RUN make -j && make install && cd .. && rm -rf AriaCoda


WORKDIR /home/base_controller/src
RUN rosdep init && \
    rosdep update && \
    rosdep fix-permissions && \
    rosdep install -i --from-paths urg_node2 rosaria2 rplidar_ros --rosdistro=jazzy -y

RUN sed -i 's/params_ether.yaml/params_serial.yaml/' /home/base_controller/src/urg_node2/launch/urg_node2.launch.py

WORKDIR /home/base_controller
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /home/base_controller/install/setup.bash" >> /root/.bashrc && \
    echo "alias c='clear'" >> /root/.bashrc && \
    apt clean && rm -rf /var/lib/apt/lists/*

COPY config/cyclonedds.xml /
COPY config/cartographer_config.lua /opt/ros/jazzy/share/cartographer/configuration_files

RUN sed -i 's/#PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    echo "root:docker"|chpasswd

EXPOSE 22
ENTRYPOINT ["bash", "-c", "service ssh restart && exec bash"]
