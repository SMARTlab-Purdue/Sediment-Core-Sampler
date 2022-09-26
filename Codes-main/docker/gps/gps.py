#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, Float32

import serial
from threading import Thread
from datetime import datetime
import time

import math


class SerialConnect(Thread):

	'''
	Summary:
	SerialConnect establishes the serial connection to the arduino to read the GPS and the compass data.
	The class inherits Thread class to run the serial connection process in a seperate theard.

	ser - serial connection with the arduino
	mBuffer - buffer containing the data received from serial connection
	mEnd - variable to keep track of end of the process
	posLat - latitude data received from the GPS
	posLng - longitude data received from the GPS
	angle - angle received from the compass
	windspeed - windspeed data received from the Anemometer
	winddirection - winddirection data received from the Anemometer
	'''

	def __init__(self):
		super(SerialConnect, self).__init__()
		self.ser = serial.Serial("/dev/wabash_gps", 9600, timeout=1)
		self.mBuffer = ""
		self.mEnd = False

	def terminate():
		self.mEnd = True

	def stop(self):
		self._stop.set()

	def stopped(self):
		return self._stop.isSet()

	def run(self):
		self.posLat = 0.
		self.posLng = 0.
		self.angle = 0
		self.windspeed = 0.
		self.winddirection = 0
		input_format = "latitude,longitude@headangle@windspeed,winddirection"
		while (not self.mEnd):
			try:
				self.mBuffer = self.ser.readline().decode()
				gps, ang, wind = self.mBuffer.split("@")
				lat, lng = gps.split(",")
				self.posLat = float(lat)
				self.posLng = float(lng)
				self.angle = int(ang)
				windspeed, winddirection = wind.split(",")
				self.windspeed = float(windspeed)
				self.winddirection = int(winddirection)
			except Exception as ex:
				print("Could not parse %s: %s", self.mBuffer, str(ex) )
				print("Expected format is %s", input_format)
				#break

class gps():
	def __init__(self):
		rospy.init_node('gps', anonymous=True)
		self.gps_pub = rospy.Publisher('/usv/gps', NavSatFix, queue_size=10)
		self.head_pub = rospy.Publisher('/usv/heading', Int32, queue_size=10)
		self.windspeed_pub = rospy.Publisher('/usv/wind/speed', Float32, queue_size=10)
		self.winddirection_pub = rospy.Publisher('/usv/wind/direction', Int32, queue_size=10)

	def spin(self):
		'''
		Summary:
		Keeps the node alive and active.

		Parameters:
		None

		Returns:
		None
		'''
		self.serial_con = SerialConnect()
		self.serial_con.start()

		time.sleep(3)
		rospy.loginfo("GPS: Initialized...")

		rate = rospy.Rate(15)
		while not rospy.is_shutdown():
			gp = NavSatFix()
			gp.latitude = self.serial_con.posLat
			gp.longitude = self.serial_con.posLng
			self.gps_pub.publish(gp)
			self.head_pub.publish(self.serial_con.angle)
			self.windspeed_pub.publish(self.serial_con.windspeed)
			self.winddirection_pub.publish(self.serial_con.winddirection)
			rate.sleep()

if __name__ == '__main__':
	try:
		ic = gps()
		ic.spin()
	except rospy.ROSInterruptException:
		pass
