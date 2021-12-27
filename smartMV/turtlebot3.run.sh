#!/bin/bash
source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup nav2_simulation_launch.py
