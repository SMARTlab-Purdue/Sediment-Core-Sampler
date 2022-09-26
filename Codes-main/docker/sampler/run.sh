#!/bin/bash

echo "this is under development. Cannot use"
echo "Exiting..."
exit 1

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
NAME=wabash_motor_driver
MOTOR_PORT=/dev/wabash_motor_driver
MOTOR_BAUD=115200

docker rm -f ${NAME}

docker run -d \
  --name ${NAME} \
  --network host \
  --restart always \
  --env ROS_IP=${MY_IP} \
  --env ROS_MASTER_URI=http://${MASTER_IP}:11311 \
  --device ${MOTOR_PORT} \
  -v $(pwd)/controller.py:/app/controller.py \
  wabash_ros \
  python3 -u /app/controller.py
