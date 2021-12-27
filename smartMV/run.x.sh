#!/bin/bash
pwd=`pwd`
cpus_num=$(grep processor /proc/cpuinfo | wc -l)
echo ${cpus_num}
cpustart=$((cpus_num / 2))
cpusend=$((cpus_num -1 ))
echo ${cpustart}
echo ${cpusend}
mkdir -p /tmp/niuma_ros
docker run \
  -u $(id -u ${USER}):$(id -g ${USER}) \
  -e "DISPLAY=$DISPLAY" \
  --group-add dialout \
  --group-add audio \
  --group-add video \
  -v ${PWD}:${PWD} \
  -v /etc/passwd:/etc/passwd \
  -v /etc/group:/etc/group \
  -v /tmp/niuma_ros:/tmp \
  -v ${HOME}:${HOME} \
  --privileged \
  --workdir="${PWD}"\
  --net=host \
  --cpuset-cpus=${cpustart}-${cpusend} \
  -it niuma_navi2 /bin/bash 
