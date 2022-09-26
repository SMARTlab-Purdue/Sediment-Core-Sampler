#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from config import *
from waypoints import *

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
    '''
    def __init__(self):
        super(SerialConnect, self).__init__()
        self.ser = serial.Serial(GPS_PORT, BAUD_RATE, timeout=1)
        self.mBuffer = ""
        self.mEnd = False

    def terminate():
        self.mEnd = True

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        while (not self.mEnd):
            try :
                self.mBuffer = self.ser.readline()
                if "@" in self.mBuffer:
                    gps, ang = self.mBuffer.split("@")

                    if "," in gps:
                        lat, lng = gps.split(",")
                        self.posLat = float(lat)
                        self.posLng = float(lng)

                    if "." in ang:
                        a = ang.split(".")
                        self.angle = int(a[0])
            except serial.serialutil.SerialException:
                pass


class auto_control():
    def __init__(self):
        rospy.init_node('auto_control', anonymous=True)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.kdp = rospy.get_param('~Kdp', 0.05)
        self.kap = rospy.get_param('~Kap', 0.8)
        self.kad = rospy.get_param('~Kad', 0.0)

        self.theta_e_old = 0
	self.d_e = 6
	self.goal = False
	self.new_file = False
	rospack = rospkg.RosPack()
	self.pkg_path = rospack.get_path('smartUSV2')
	self.master_log = self.pkg_path + "/log/" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S")+"_complete.csv"

    def haversine(self, lat1, lon1, lat2, lon2):
        '''
        Summary:
        Computes the distance between two GPS coordinates.

        Parameters:
        lon1 - longitude point 1
        lat1 - latitude point 1
        lon2 - longitude point 2
        lat2 - latitude point 2

        Returns:
        None
        '''
        lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        km = 6367 * c
        return km * 1000 # in m

    def desireAngle(self, lat1, lon1, lat2, lon2):
        '''
        Summary:
        Computes the angle between two GPS coordinates.

        Parameters:
        lon1 - longitude point 1
        lat1 - latitude point 1
        lon2 - longitude point 2
        lat2 - latitude point 2

        Returns:
        None
        '''
        lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
        dlon = lon2 - lon1
        dphi = math.log(math.tan(lat2 / 2. + math.pi / 4.) / math.tan(lat1 / 2. + math.pi / 4.))
        if abs(dlon) > math.pi:
            if dlon > 0.0:
                dlon = -(2. * math.pi - dlon)
            else:
                dlon = (2. * math.pi + dlon)
        return (math.degrees(math.atan2(dlon, dphi)) + 360.) % 360.  #convert to compass bearing

    
    def write_file(self, file_name, lat, lon):
	'''
        Summary:
        Write GPS coordinates to a .csv file.

        Parameters:
        lon - longitude point 
        lat - latitude point 
	file_name

        Returns:
        None
        '''
        # print ("In write_temp function - "+file_name)

        with open(file_name, 'a') as log:
                log.write("{0},{1},{2}\n".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S"),str(lat), str(lon)))


    def navigate(self):
            self.d_e = self.haversine(self.serial_con.posLat, self.serial_con.posLng, self.destination[0], self.destination[1])
	    # print (self.serial_con.posLat, self.serial_con.posLng, self.destination[0], self.destination[1], self.d_e)
	    rospy.loginfo(" CurLat: " +  str(self.serial_con.posLat) + " CurLng: " +  str(self.serial_con.posLng) + " CurAng: " +  str(self.serial_con.angle) + " CurTime: " + str(datetime.now()))
            des_theta = self.desireAngle(self.serial_con.posLat, self.serial_con.posLng, self.destination[0], self.destination[1])
            theta_e = des_theta - int(self.serial_con.angle) + 180 # +180 to revert the direction
            print "The theta error is " + str(theta_e)
            while (theta_e > 180):
                theta_e -= 360
            while (theta_e < -180):
                theta_e += 360

            linSpd = self.kdp * self.d_e
            rotSpd = self.kap * theta_e + self.kad * (theta_e - self.theta_e_old)
            self.theta_e_old = theta_e

            rotSpd = math.pi * rotSpd / 180

	    if self.new_file:
	    	self.filename = self.pkg_path + "/log/" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S")+".csv"
		self.new_file = False
	    self.write_file(self.filename, self.serial_con.posLat, self.serial_con.posLng)

            twist = Twist()
            if self.joy_data.buttons[7] == 0:
                linSpd = 0
                rotSpd = 0
	    # rospy.loginfo("Linear: " + str(linSpd) + " Angular: " + str(rotSpd))
            twist.linear.x = linSpd
            twist.angular.z = rotSpd
            self.vel_pub.publish(twist)
            if self.d_e <= 5.0:
		self.goal = False
		self.new_file = True

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

	time.sleep(30)
	rospy.loginfo("Auto Control: Initialized...")

        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
	    self.write_file(self.master_log, self.serial_con.posLat, self.serial_con.posLng)
	    if self.goal:
	    	self.navigate()
            rate.sleep()

    def joy_callback(self, data):
        '''
        Summary:
        Call back for joy updates from joy node.

        Parameters:
        data (Joy): Button and axis readings from the Joystick

        Returns:
        None
        '''
        self.joy_data = data

        if data.buttons[0] == 1:
	    self.goal =  True
	    self.new_file = True
            self.destination = WAY_POINT_1

	if data.buttons[2] == 1:
	    self.goal =  True
	    self.new_file = True
            self.destination = WAY_POINT_2

	if data.buttons[3] == 1:
	    self.goal =  True
	    self.new_file = True
            self.destination = HOME

	if data.buttons[1] == 1:
            self.goal = False  #abort

	
if __name__ == '__main__':
    try:
        ic = auto_control()
        ic.spin()
    except rospy.ROSInterruptException:
        pass
