#!/bin/bash

if [ -z ${lat} ] || [ -z ${lon} ]; then
  echo "No destination is given. Halting..."
  echo "Usage: lat=LATITUDE lon=LONGITUDE auto.sh"
  exit 1
fi

docker run -i --rm \
  --network host \
  --env ROS_IP=${MY_IP} \
  --env ROS_MASTER_URI=http://${MASTER_IP}:11311 \
  wabash_ros /bin/bash <<EOF
source /opt/ros/melodic/setup.bash
echo Setting the autopilot to a coordinate: "${lat}", "${lon}"
set -x
rostopic pub -1 /usv/autopilot/cmd/engage std_msgs/Bool True
rostopic pub -1 /usv/autopilot/destination sensor_msgs/NavSatFix '{latitude: ${lat}, longitude: ${lon}}'
set +x
echo Done
EOF

