ARG BASE_OS=ros:jazzy-ros-base
FROM ${BASE_OS}

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
RUN apt-get update && apt-get full-upgrade -y && \
    apt-get install -y --no-install-recommends \
        git wget openssh-server vim curl nano python3-pip \
        software-properties-common \
        doxygen \
        libboost-dev \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-twist-mux \
        ros-jazzy-cartographer-ros \
        ros-jazzy-tf-transformations \
        ros-dev-tools \
        ros-jazzy-teleop-twist-joy \
        ros-jazzy-joy 

WORKDIR /home/jaime_ws/src
RUN git clone https://github.com/IntelligentRoboticsLabs/kobuki.git  -b jazzy && \
    git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git && \
    git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git && \ 
        # Uchile dependencies
    git clone -b feat-jazzy https://github.com/uchile-robotics/jaime_bringup.git && \
    git clone -b feat-jazzy https://github.com/uchile-robotics/jaime_core.git && \
    git clone -b feat-jazzy https://github.com/uchile-robotics/uchile_navigation.git
    

WORKDIR /home/jaime_ws/src
RUN rosdep install -i --from-paths . --rosdistro=jazzy -y

RUN sed -i 's/params_ether.yaml/params_serial.yaml/' /home/jaime_ws/src/urg_node2/launch/urg_node2.launch.py



RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /home/jaime_ws/install/setup.bash" >> /root/.bashrc && \
    echo "alias c='clear'" >> /root/.bashrc && \
    apt clean && rm -rf /var/lib/apt/lists/*

COPY config/cyclonedds.xml /

RUN sed -i 's/#PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    echo "root:docker"|chpasswd

EXPOSE 22
ENTRYPOINT ["bash", "-c", "service ssh restart && exec bash"]
