#!/bin/bash
pwd=`pwd`
cpus_num=$(grep processor /proc/cpuinfo | wc -l)
echo ${cpus_num}
cpustart=$((cpus_num / 2))
cpusend=$((cpus_num -1 ))
echo ${cpustart}
echo ${cpusend}
docker run \
  -p 6080:80 --shm-size=512m \
  tiryoh/ros2-desktop-vnc:foxy
