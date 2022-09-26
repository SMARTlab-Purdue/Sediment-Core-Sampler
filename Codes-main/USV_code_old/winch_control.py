#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy

# Motor driver- SmartDriveDuo-60
import webiopi
from webiopi.devices.serial import Serial

from config import *

class winch_control():
    def __init__(self):
   	rospy.init_node('winch_control', anonymous=True)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.serial = Serial(WINCH_PORT, BAUD_RATE)

    def spin(self):
        '''
        Summary:
        Keeps the node alive and active.

        Parameters:
        None

        Returns:
        None
        '''
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def joy_callback(self, data):
        '''
        Summary:
        Call back for joy updates from joy node and lower and raise the winch.

        Parameters:
        data (Joy): Button and axis readings from the Joystick

        Returns:
        None
        '''
        if (data.axes[1] == 1.0):
            rospy.loginfo("WINCH RAISING")
            speed = bytearray([WINCH_SPEED_BACK])
            self.serial.writeString(speed)

        if (data.axes[1] == -1.0):
            rospy.loginfo("WINCH LOWERING")
            speed = bytearray([WINCH_SPEED_FORW])
            self.serial.writeString(speed)

        if (data.axes[1] == 0.0):
            speed = bytearray([0])
            self.serial.writeString(speed)


if __name__ == '__main__':
    try:
        ic = winch_control()
        ic.spin()
    except rospy.ROSInterruptException:
        pass
