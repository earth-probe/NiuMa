#!/bin/bash
pwd=`pwd`
cpus_num=$(grep processor /proc/cpuinfo | wc -l)
echo ${cpus_num}
cpustart=$((cpus_num / 2))
cpusend=$((cpus_num -1 ))
echo ${cpustart}
echo ${cpusend}
docker run \
  -u $(id -u ${USER}):$(id -g ${USER}) \
  --group-add dialout \
  --group-add audio \
  --group-add video \
  -v $(pwd):$(pwd) \
  -v /etc/passwd:/etc/passwd \
  -v /etc/group:/etc/group \
  -v $(HOME):$(HOME) \
  --privileged \
  --workdir="$(pwd)"\
  --net=host \
  --cpuset-cpus=${cpustart}-${cpusend} \
	ros:foxy /bin/bash 
