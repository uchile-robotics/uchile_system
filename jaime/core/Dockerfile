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
        ros-jazzy-joy \
        ros-jazzy-kobuki-ros-interfaces 


#intento instalar gazebo ###########

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
     && wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
     && sudo apt-get update -qq \
     && sudo apt-get install gazebo11 libgazebo11-dev -y

####################################
WORKDIR /home/jaime_ws/src
RUN git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git && \
    git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git && \ 
        # Uchile dependencies
    git clone -b feat-jazzy https://github.com/uchile-robotics/jaime_bringup.git && \
    git clone -b feat-jazzy https://github.com/uchile-robotics/jaime_core.git && \
    git clone -b feat-jazzy https://github.com/uchile-robotics/uchile_navigation.git &&\
    git clone -b jazzy https://github.com/IntelligentRoboticsLabs/kobuki.git &&\
    git clone -b devel https://github.com/Juancams/ecl_core.git &&\
    git clone -b ros2-jazzy https://github.com/Juancams/ecl_lite.git &&\
    git clone -b devel https://github.com/kobuki-base/kobuki_core.git &&\
    git clone -b jazzy-devel https://github.com/Juancams/kobuki_ros.git 


#cosas que son para simular nomas pero si no colapsa cuando se instala el docker

WORKDIR /home/jaime_ws/src
RUN git clone -b ros2-jazzy https://github.com/Juancams/aws-robomaker-bookstore-world.git &&\
    git clone -b ros2-jazzy https://github.com/Juancams/aws-robomaker-racetrack-world.git &&\
    git clone -b ros2-jazzy https://github.com/Juancams/aws-robomaker-small-house-world.git &&\
    git clone -b ros2-jazzy https://github.com/Juancams/aws-robomaker-small-warehouse-world.git &&\
    git clone -b ros2 https://github.com/ros-drivers/openni2_camera.git &&\
    git clone -b rolling https://github.com/Juancams/ros_astra_camera.git

WORKDIR /home/jaime_ws/src
RUN rosdep install -i --from-paths . --skip-keys gazebo_plugins --rosdistro=jazzy -y

WORKDIR /home/jaime_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --packages-ignore aws_robomaker_small_house_world --symlink-install"

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /home/jaime_ws/install/setup.bash" >> /root/.bashrc && \
    echo "alias c='clear'" >> /root/.bashrc && \
    apt clean && rm -rf /var/lib/apt/lists/*


COPY config/cyclonedds.xml /

RUN sed -i 's/#PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    echo "root:docker"|chpasswd

EXPOSE 22
ENTRYPOINT ["bash", "-c", "service ssh restart && exec bash"]
