#!/bin/bash -x

source setup.sh

cd rosserial
./run.sh

cd ../rotary_controller
./run.sh

cd ../solenoid_controller
./run.sh

cd ../distance_traveled
./run.sh

#cd ../pressure_depth
#./run.sh

cd ../usvmotor
./run.sh

cd ../gps
./run.sh

cd ../usv_autopilot
./run.sh

cd ../camera
./video_server.run
sleep 5
./camera.run
./camera_2.run
