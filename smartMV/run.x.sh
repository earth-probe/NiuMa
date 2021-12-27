#!/bin/bash
pwd=`pwd`
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
  -it niuma_navi2 /bin/bash 
