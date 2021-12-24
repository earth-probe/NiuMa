FROM ros:foxy
MAINTAINER watorvapor@gmail.com
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-turtlebot3* && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get -y update && apt-get install -y \
    gazebo11 \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros && \
    rm -rf /var/lib/apt/lists/*

