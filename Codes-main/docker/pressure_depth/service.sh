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
NAME=wabash_pressure_depth
PRESSURE_PORT=/dev/wabash_pressure_device
PRESSURE_BAUD=9600

docker rm -f ${NAME}

docker run --rm \
  --name ${NAME} \
  --network host \
  --env ROS_IP=${MY_IP} \
  --env ROS_MASTER_URI=http://${MASTER_IP}:11311 \
  --entrypoint /usr/bin/python3 \
  --device ${PRESSURE_PORT} \
  -v $(pwd)/pressure_depth.py:/app/pressure_depth.py \
  wabash_ros \
  /app/pressure_depth.py
