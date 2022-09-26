import serial

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
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
		self.rate = rospy.get_param("~rate", 60)
		self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
		self.left = 0
		self.right = 0
		self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.twistCallback)
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
		self.left = (1.5 * self.dx - self.dr * self.w) / 2.0
		self.right = (1.5 * self.dx + self.dr * self.w) / 2.0

		# rospy.loginfo("Right Speed: " + str(self.right) + " Left Speed: " + str(self.left))

		# Motor 1 - Right Motor
		motor_1 = 0.0

		# forward Thrust
		if self.right > 0:
			lower, upper = 128.0, 255.0
			motor_1 = self.right * (upper - lower) + lower
			# motor_1 = lower + (upper - lower) * self.right
			if motor_1 > upper:
				motor_1 = upper

		# reverse Thrust
		if self.right < 0:
			lower, upper = 0, 126
			motor_1 = lower + (upper-lower) * abs(self.right)
			motor_1 = abs(motor_1-upper)
			if motor_1 > upper:
				motor_1 = upper

		# neutral
		if self.right == 0:
			motor_1 = 127

		motor_1 = int(motor_1)

		# Motor 2 - Left Motor
		motor_2 = 0.0

		# forward thrust
		if self.left > 0:
			lower, upper = 128.0, 255.0
			motor_2 = self.left * (upper - lower) + lower
			if motor_2 > upper:
				motor_2 = upper

		# reverse thrust
		if self.left < 0:
			lower, upper = 0.0, 126.0
			motor_2 = lower + (upper - lower) * abs(self.left)
			motor_2 = abs(motor_2-upper)
			if motor_2 > upper:
				motor_2 = upper

		# neutral
		if self.left == 0:
			motor_2 = 127

		motor_2 = int(motor_2)

		anchor_stat  = rospy.wait_for_message('/anchor', Int32)
		anchor_stat = anchor_stat.data

		# USV moves only when the anchor is neutral
		if (anchor_stat == 0):
			# rospy.loginfo("Motor 1: " + str(motor_1) + " Motor 2: " + str(motor_2))
			msg = "th l " + str(motor_2) + "\r\n"
			self.serial.writeString(msg)

			msg = "th r " + str(motor_1) + "\r\n"
			self.serial.writeString(msg)

		self.ticks_since_target += 1

if __name__ == '__main__':
	try:
		ic = motor_control()
		ic.spin()
	except KeyboardInterrupt:
		print("Shutting down")

import rospy
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Int32, Bool

import time

# Motor driver- SmartDriveDuo-60
import webiopi
from webiopi.devices.serial import Serial

from config import *

class sampler_control():
	def __init__(self):
		rospy.init_node('sampler_control', anonymous=True)
		self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1)
		self.imu_sub = rospy.Subscriber('/sampler/imu', Imu, self.imu_callback, queue_size=1)

		self.serial = Serial(MOTOR_PORT, BAUD_RATE)

		self.sampler_status_pub = rospy.Publisher('/sampler', Int32, queue_size=10)
		self.sampler_stable_pub = rospy.Publisher('/sampler_stable', Bool, queue_size=10)

		self.sampler_status = 0
		self.sampler_stable_status = False

		self.stable_count = 0

		self.motor_stop = True


	def spin(self):
		'''
		Summary:
		Keeps the node alive and active.

		Parameters:
		None

		Returns:
		None
		'''
		rate = rospy.Rate(20)
		while not rospy.is_shutdown():
			self.sampler_status_pub.publish(self.sampler_status)
			rate.sleep()

	def imu_callback(self, data):
		ang_vel = data.angular_velocity
		if ang_vel.x == 0 and ang_vel.y == 0 and ang_vel.z == 0:
			self.stable_count += 1
		else:
			self.stable_count = 0
			self.sampler_stable_status = False

		if self.stable_count > 20:
			self.sampler_stable_status  = True

		self.sampler_stable_pub.publish(self.sampler_stable_status)

	def joy_callback(self, data):
		'''
		Summary:
		Call back for joy updates from joy node and lower and raise the anchor.

		Parameters:
		data (Joy): Button and axis readings from the Joystick

		Returns:
		None
		'''
		if (data.axes[7] == 1.0 and data.buttons[4] == 1.0):
			# raising sampler
			self.motor_stop = False
			self.sampler_status = 2
			msg = "sa r 192\r\n"
			self.serial.writeString(msg)

		if (data.axes[7] == -1.0 and data.buttons[4] == 1.0):
			# lowering sampler
			rospy.loginfo("Lowering")
			self.motor_stop = False
			self.sampler_status = 1
			msg = "sa r 63\r\n"
			self.serial.writeString(msg)

		if (data.axes[7] == 0.0 and data.buttons[4] == 1.0):
			# neutral
			if not self.motor_stop:
				self.sampler_status = 0
				msg = "sa r 127\r\n"
				self.serial.writeString(msg)
				self.motor_stop =  True

if __name__ == '__main__':
	try:
		ic = sampler_control()
		ic.spin()
	except rospy.ROSInterruptException:
		pass
