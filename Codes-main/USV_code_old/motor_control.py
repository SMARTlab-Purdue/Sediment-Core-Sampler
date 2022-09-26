#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

#Motor driver- Saber tooth 2 x32 A
import webiopi
from webiopi.devices.serial import Serial

from time import sleep

from config import *


class motor_control():
    '''
    Summary:
    motor_control class to control the driving motors.
    Subsribes to the command velocity data from the joystick or the autonomous controller
    and converts to left and right motor signals.
    Uses serial connection to send the signals to the motors.

    rate - rate of the ROS looping
    timeout_ticks - number of ticks after which to stop publishing
    left - speed of the left motor
    right - speed of the right motor
    cmd_vel_sub - subsriber to command velocity
    serial - serial connection to the motor driver (check the config for port and baud rate)
    '''
    def __init__(self):
   	rospy.init_node('motor_control', anonymous=True)
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.twistCallback)
	self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.serial = Serial(MOTOR_PORT, BAUD_RATE)
	self.toggle_enabled = False

    def twistCallback(self,msg):
        '''
        Summary:
        Call back for Twist data generated using joystick or autonmous controller.

        Parameters:
        data (Twist): Linear and angular velocity for driving the USV.

        Returns:
        None
        '''
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z

    def spin(self):
        '''
        Summary:
        Loop to stop sending data to motor controller when no cmd_vel updates.

        Parameters:
        None.

        Returns:
        None
        '''
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks

        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()

    def joy_callback(self, data):
        '''
        Summary:
        Call back for joy updates from joy node and lower and raise the winch.

        Parameters:
        data (Joy): Button and axis readings from the Joystick

        Returns:
        None
        '''
	
	right = data.axes[2]
	left = data.axes[5]
	left_speed = 0
	right_speed = 0

	if right == -1.0 and left == -1.0:
		self.toggle_enabled = True
	if self.toggle_enabled:
		left = (left - 1.0) * (-0.5)
		right = (right - 1.0) * (-0.5)
		left_speed = int(left * MAX_MOTOR_SPEED)
		right_speed = int(-1.0 * (right * MAX_MOTOR_SPEED))

		motor_1_msg = "M1: "+str(right_speed)+"\r\n"
		motor_2_msg = "M2: "+str(left_speed)+"\r\n"
		
	if (not left_speed == 0 or not right_speed == 0) and self.toggle_enabled:
		self.serial.writeString(motor_1_msg)
		self.serial.writeString(motor_2_msg)


    def spinOnce(self):
        '''
        Summary:
        Converts the linear and angular velocities to left and right motor speeds.
        Sends the speeds to the left and right motor using serial connection.

        Parameters:
        None.

        Returns:
        None
        '''
        self.w = 0.5
        self.left = (1.5 * self.dx + self.dr * self.w) / 2
        self.right = (1.5 * self.dx - self.dr * self.w) / 2

        # Motor 1
        motor_1 = int(self.right * MAX_MOTOR_SPEED)
        if motor_1 > MAX_MOTOR_SPEED:
            motor_1 = MAX_MOTOR_SPEED

        if motor_1 < -MAX_MOTOR_SPEED:
            motor_1 = -MAX_MOTOR_SPEED

	motor_1 = -1 * motor_1
        motor_1_msg = "M1: "+str(motor_1)+"\r\n"
        

        # Motor 2
        motor_2 = int(self.left * MAX_MOTOR_SPEED)
        if motor_2 > MAX_MOTOR_SPEED:
            motor_2 = MAX_MOTOR_SPEED

        if motor_2 < -MAX_MOTOR_SPEED:
            motor_2 = -MAX_MOTOR_SPEED

	
	motor_2_msg = "M2: "+str(motor_2)+"\r\n"
	self.serial.writeString(motor_1_msg)
        self.serial.writeString(motor_2_msg)

        # rospy.loginfo("Right: " + str(motor_1) + " Left: " + str(motor_2))
        self.ticks_since_target += 1

if __name__ == '__main__':
    try:
        ic = motor_control()
        ic.spin()
    except KeyboardInterrupt:
        print("Shutting down")
