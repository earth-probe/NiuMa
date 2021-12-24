FROM ros:foxy
MAINTAINER watorvapor@gmail.com
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update && apt-get install -y \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/ros-planning/moveit2_tutorials.git -b foxy
RUN vcs import < moveit2_tutorials/moveit2_tutorials.repos
RUN rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
