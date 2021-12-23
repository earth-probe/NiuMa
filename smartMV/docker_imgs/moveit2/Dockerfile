FROM ros:foxy
MAINTAINER watorvapor@gmail.com
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update && \
    ros-${ROS_DISTRO}-ros-tutorials \
    ros-${ROS_DISTRO}-common-tutorials && \
    rm -rf /var/lib/apt/lists/*
