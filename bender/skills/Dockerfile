ARG BASE_OS=osrf/ros:jazzy-desktop
FROM ${BASE_OS}

RUN apt update && apt install -y ros-jazzy-rmw-cyclonedds-cpp
COPY config/cyclonedds.xml /
