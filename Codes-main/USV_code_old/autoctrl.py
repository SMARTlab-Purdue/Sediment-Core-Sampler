#! /usr/bin/env python
# By Shaocheng Luo, 08/01/2019
# For USV version_2 automonous control. 
# The code reads GPS & Compass info from Ardu Uno via serial, outputs /cmd_vel (linSpd and rotSpd)

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

import sys, select, termios, tty

import asyncore
import asynchat
import socket

import serial
import threading
from threading import Thread

import math
import datetime
import wp

HOST = '192.168.1.53'
PORT = 12345
BUFSIZE = 1024
ADDR = (HOST, PORT)

# waypoint sets has index 1, 2 and 3
dest = wp.WP_SET0


global posLng, posLat, angle
global theta_e_old # Old (previous) angle error
global enable_cmd_vel_pub
theta_e_old = 0
posLat = 0
posLng = 0
angle = 0
enable_cmd_vel_pub = 0
		
"""
Serial communication to read GPS and Compass
"""
class MySerial(Thread):
	def __init__(self):
		super(MySerial, self).__init__()
		self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
		self.mBuffer = ""
		self.mEnd = False
		
	def terminate():
		self.mEnd = True

	def stop(self):
		self._stop.set()

	def stopped(self):
		return self._stop.isSet()

	def run(self):
		global posLng, posLat, angle
		while (not self.mEnd):
			try :
				self.mBuffer = self.ser.readline()
				#print self.mBuffer
				if "@" in self.mBuffer:
					gps, ang = self.mBuffer.split("@")
					if "," in gps:
						#print len(gps)
						lat, lng = gps.split(",")
						posLat = float(lat)
						posLng = float(lng)
						# print posLat
						# print posLng
					if "." in ang:
						a = ang.split(".")
						angle = int(a[0])
						# print angle

			except serial.serialutil.SerialException:
				pass
				



"""
Calculate distance between GPSs
"""
def haversine(lat1, lon1, lat2, lon2):
	lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
	dlon = lon2 - lon1
	dlat = lat2 - lat1
	a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
	c = 2 * math.asin(math.sqrt(a))
	km = 6367 * c
	return km * 1000 # in m

def desireAngle(lat1, lon1, lat2, lon2):
	lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
	dlon = lon2 - lon1
	dphi = math.log(math.tan(lat2 / 2. + math.pi / 4.) / math.tan(lat1 / 2. + math.pi / 4.))
	if abs(dlon) > math.pi:
		if dlon > 0.0:
			dlon = -(2. * math.pi - dlon)
		else:
			dlon = (2. * math.pi + dlon)

	return (math.degrees(math.atan2(dlon, dphi)) + 360.) % 360.  #convert to compass bearing


"""
Control loop
"""
def navi(Kdp, Kap, Kad):
	global dest,posLng,posLat,angle, theta_e_old
	# P values foe distance and bearing, respectively

	
	# To see if one can move to the next waypoint
	if (len(dest) < 1):
		print "NO FURTHER WAYPOINT TO GO"
		return (0, 0)
	waypt = dest[0]
	d_e = haversine(posLat, posLng, waypt[0], waypt[1])
	
	# if the robot arrives at the way point, remove the point from the list
	print (d_e),
	if (d_e <= 5.):
		dest.pop(0)
		#print("arrive")
		return (0, 0)
	
	# Note that positive rotation is CW
	des_theta = desireAngle(posLat, posLng, waypt[0], waypt[1])
	theta_e = des_theta - int(angle)
	
	while (theta_e > 180):
		theta_e -= 360
	while (theta_e < -180):
		theta_e += 360
		
	print "Theta error is " + str(theta_e)
	linSpd = Kdp * d_e
	rotSpd = Kap * theta_e + Kad * (theta_e - theta_e_old)
	#rotSpd = Kap * theta_e
	
	theta_e_old = theta_e
	return (linSpd, rotSpd)

'''
Joy callback
'''

def joy_callback(data):    
	global enable_cmd_vel_pub
    
        if (data.buttons[4] == 1):
	    enable_cmd_vel_pub = 1
	else:
	    enable_cmd_vel_pub = 0
            



"""
Main loop
"""
def autoctrl():
	global posLng, posLat, angle, enable_cmd_vel_pub

	s = MySerial()

	try:
		s.start()
	
		rospy.init_node('autoctrl', anonymous=True)
		out_topic = rospy.get_param('~output_topic','cmd_vel')
		
		r = rospy.Rate(20)
		
		# Publisher		
		pub = rospy.Publisher(out_topic, Twist, queue_size=10)
		
		# Subscriber
		rospy.Subscriber('/joy', Joy, joy_callback)
		
		# ROS Parameters
		Kdp = rospy.get_param('~Kdp',0.02)
		Kap = rospy.get_param('~Kap',1.0)
		Kad = rospy.get_param('~Kad',0.0)		
		
		print "go auto control in loop"
		while(not rospy.is_shutdown()):
			#print "calculating"
			LinSpd, RotSpd = navi(Kdp, Kap, Kad)
			rospy.loginfo(" CurLat: " +  str(posLat) + " CurLng: " +  str(posLng) + " CurAng: " +  str(angle) +  " LinSpd: " +  str(LinSpd) + " RotSpd: " + str(RotSpd) + " CurTime: " + str(datetime.datetime.now()))
			print "Kdp" + str(Kdp) + "Kap" + str(Kap) + "Kad" + str(Kad) 
			RotSpd_r = math.pi * RotSpd / 180

			sys.stdout.flush()
			
			twist = Twist()
			twist.linear.x = LinSpd
			twist.angular.z = RotSpd_r
			
			if (enable_cmd_vel_pub == 1):
				pub.publish(twist)

			#print "ros sleeping"
			r.sleep()

			#print "go to next"
	except (KeyboardInterrupt, SystemExit):
		s.terminate()
		cleanup_stop_thread();
		sys.exit()
	
	

if __name__=='__main__':	
	try:
		autoctrl()
	except rospy.ROSInterruptException:
		pass
			
