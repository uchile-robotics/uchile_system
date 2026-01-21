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
        ros-jazzy-joy-linux \
        ros-jazzy-kobuki-ros-interfaces \
        ros-jazzy-ros-gz \
        ros-jazzy-ecl-build\
        ros-jazzy-kobuki-velocity-smoother\
        curl lsb-release gnupg\
        ros-jazzy-joint-state-publisher-gui
        

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg &&\
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null &&\
    apt-get update &&\
    apt-get install gz-harmonic -y


WORKDIR /home/jaime_ws/src
RUN git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git &&\
    git clone -b ros2 https://github.com/Juancams/rplidar_ros.git &&\
    git clone -b feat-jazzy https://github.com/uchile-robotics/jaime_bringup.git &&\
    git clone -b feat-jazzy https://github.com/uchile-robotics/jaime_core.git &&\
    git clone -b feat-jazzy https://github.com/uchile-robotics/uchile_navigation.git &&\
    git clone -b jazzy https://github.com/IntelligentRoboticsLabs/kobuki.git &&\
    git clone -b devel https://github.com/Juancams/ecl_core.git &&\
    git clone -b ros2-jazzy https://github.com/Juancams/ecl_lite.git &&\
    git clone -b devel https://github.com/kobuki-base/kobuki_core.git &&\
    git clone -b jazzy-devel https://github.com/Juancams/kobuki_ros.git 


#cosas que son para simular nomas pero si no colapsa cuando se instala el docker

WORKDIR /home/jaime_ws/src
RUN git clone -b ros2 https://github.com/ros-drivers/openni2_camera.git 

WORKDIR /home/jaime_ws/src
RUN rosdep init || true &&\
    rosdep update &&\
    rosdep install -i --from-paths . \
    --skip-keys="gazebo_plugins gazebo_ros gazebo_msgs aws_robomaker_small_house_world gazebo aws_robomaker_bookstore_world aws_robomaker_racetrack_world aws_robomaker_small_warehouse_world astra_camera astra_camera_msgs \
    ros-jazzy-aws-robomaker-small-warehouse-world ros-jazzy-aws-robomaker-bookstore-world ros-jazzy-aws-robomaker-racetrack-world ros-jazzy-aws-robomaker-small-warehouse-world" -r --rosdistro=jazzy -y

WORKDIR /home/jaime_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --packages-ignore aws_robomaker_small_house_world jaime_testing astra_camera aws-robomaker-bookstore-world aws-robomaker-racetrack-world aws-robomaker-small-warehouse-world --symlink-install"

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /home/jaime_ws/install/setup.bash" >> /root/.bashrc && \
    echo "alias c='clear'" >> /root/.bashrc && \
    apt-get clean && rm -rf /var/lib/apt/lists/*


COPY config/cyclonedds.xml /

RUN sed -i 's/#PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    echo "root:docker"|chpasswd

RUN apt-get update && apt-get install -y \
    x11-apps \
    libx11-6 libx11-xcb1 libxcb1 \
    libxcb-render0 libxcb-render-util0 \
    libxcb-shape0 libxcb-xfixes0 \
    libxcb-xinerama0 libxcb-randr0 \
    libxcb-image0 libxcb-keysyms1 \
    libxkbcommon-x11-0 \
    qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
    qtwayland5

    
    EXPOSE 22
ENTRYPOINT ["bash", "-c", "service ssh restart && exec bash"]
 