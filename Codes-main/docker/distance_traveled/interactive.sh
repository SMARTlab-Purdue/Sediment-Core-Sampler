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
NAME=wabash_distance_traveled

docker rm -f ${NAME}

docker run -ti --rm \
  --name ${NAME} \
  --network host \
  --env ROS_IP=${MY_IP} \
  --env ROS_MASTER_URI=http://${MASTER_IP}:11311 \
  -v $(pwd)/distance_traveled.py:/app/distance_traveled.py \
  wabash_ros
