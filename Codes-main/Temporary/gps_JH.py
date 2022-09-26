#!/usr/bin/env python 
import rospy
import rospkg
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32

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
windspeed - windspeed data received from the Anemometer          - added
winddirection - winddirection data received from the Anemometer  - added 
'''

def __init__(self):
super(SerialConnect, self).__init__()
self.ser = serial.Serial("/dev/wabash_gps", 115200, timeout=1) '''changed'''
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
self.windspeed = 0. '''added'''
self.winddirection = 0 '''added'''
while (not self.mEnd):
try :
self.mBuffer = self.ser.readline().decode()
if "@" in self.mBuffer:
gps, ang = self.mBuffer.split("@")
ang, wind = self.mBuffer.split("@")  '''added'''
if "," in gps:
lat, lng = gps.split(",")
self.posLat = float(lat)
self.posLng = float(lng)
if "." in ang:
reading = ang.split(".")  '''changed'''
self.angle = int(reading)  '''changed'''
if "," in wind:                      '''added'''
WindSpeed, CalDirection = gps.split(",")  '''added'''
self.windspeed = float(WindSpeed)  '''added'''
self.winddirection = int(CalDirection)  '''added'''
except serial.serialutil.SerialException:
pass

class gps():
def __init__(self):
rospy.init_node('gps', anonymous=True)
self.gps_pub = rospy.Publisher('/usv/gps', NavSatFix, queue_size=10)
self.head_pub = rospy.Publisher('/usv/heading', Int32, queue_size=10)
self.wind_pub = rospy.Publisher('/usv/gps', NavSatFix, queue_size=10)   '''added'''
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
wind = NavSatFix()                               '''added'''
wind.speed = self.serial_con.windspeed           '''added'''
wind.direction = self.serial_con.winddirection   '''added'''
self.wind_pub.publish(wind)                       '''added'''
rate.sleep()

if __name__ == '__main__':
try:
ic = gps()
ic.spin()
except rospy.ROSInterruptException:
pass
