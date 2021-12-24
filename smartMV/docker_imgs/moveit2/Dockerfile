FROM ros:foxy
MAINTAINER watorvapor@gmail.com
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update && apt-get install -y \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/ros-planning/moveit2_tutorials.git -b foxy
