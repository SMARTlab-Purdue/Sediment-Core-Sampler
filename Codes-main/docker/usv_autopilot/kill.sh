#! /bin/bash

docker run -i --rm \
  --network host \
  --env ROS_IP=${MY_IP} \
  --env ROS_MASTER_URI=http://${MASTER_IP}:11311 \
  wabash_ros /bin/bash <<EOF
source /opt/ros/melodic/setup.bash
echo Attempting to kill autopilot...
set -x
rostopic pub -1 /usv/autopilot/cmd/kill std_msgs/Bool True
set +x
EOF