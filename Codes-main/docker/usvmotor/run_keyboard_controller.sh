#!/bin/bash

echo "To install, run apt-get install ros-melodic-teleop-twist-keyboard"

# teleop_twist_keyboard looks for /cmd_vel topic
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
