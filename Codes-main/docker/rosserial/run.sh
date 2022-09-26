#!/bin/bash

if [ -z ${MY_IP} ]; then
  echo "MY_IP variable is not set!"
  exit 1
fi
if [ -z ${MASTER_IP} ]; then
  echo "MASTER_IP variable is not set!"
  exit 1
fi
echo "MY_IP: "${MY_IP}
echo "MASTER_IP: "${MASTER_IP}
NAME=wabash_sampler_serial
SAMPLER_PORT=/dev/wabash_sampler
SAMPLER_BAUD=115200

docker rm -f ${NAME}

docker run -d \
  --name ${NAME} \
  --network host \
  --restart always \
  --env ROS_IP=${MY_IP} \
  --env ROS_MASTER_URI=http://${MASTER_IP}:11311 \
  --device ${SAMPLER_PORT} \
  wabash_ros \
  rosrun rosserial_python serial_node.py _port:=${SAMPLER_PORT} _baud:=${SAMPLER_BAUD}
