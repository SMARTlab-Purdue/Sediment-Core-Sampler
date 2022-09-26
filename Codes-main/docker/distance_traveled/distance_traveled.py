#!/usr/bin/python3
import time
import threading

import rospy
from std_msgs.msg import Float32, Bool, Int32

import time

class DistanceTraveled():
    def __init__(self):
        rospy.init_node('Distance_traveled', anonymous=True)
        # calculate distance traveled
        self.distance_lock = threading.Lock()
        self.sub_reset_distance_traveled = rospy.Subscriber('/sampler/motor/linear/distance_traveled/cmd/reset', Bool, self.callback_reset_distance_traveled, queue_size=1)
        self.pub_distance_traveled = rospy.Publisher('/sampler/motor/linear/distance_traveled', Float32, queue_size=1)
        self.sub_lin_encoder = rospy.Subscriber('/sampler/encoder/linear/counter', Int32, self.callback_lin_read_encoder, queue_size=10)
        self.sub_lin_velocity = rospy.Subscriber("/sampler/motor/linear/read_velocity", Float32, self.callback_lin_read_velocity, queue_size=1)
        self.linear_velocity = 0.
        self.distance_traveled = 0.
        self.distance_last_updated = time.time()

    # reset distance traveled
    def callback_reset_distance_traveled(self, msg):
        self.distance_lock.acquire()
        self.distance_traveled = 0.
        self.distance_last_updated = -1
        self.distance_lock.release()

    # encoder ticks
    def callback_lin_read_encoder(self, msg):
        # NOTE: pitch of the thread guiding the linear motor is 0.5 mm
        #       The encoder resolution is 40 so every 40 ticks the motor drives by 0.5 mm
        resolution = 40
        pitch_length = 0.5
        ticks = msg.data

        # Determine the direction by looking at sign of the velocity of the linear motor
        d = (ticks / resolution) * pitch_length / 1.33928571 * 1.32142857 * 8.305724489890089 * 0.79091
        if self.linear_velocity > 0.:
            d *= -1
        self.distance_lock.acquire()
        self.distance_traveled += (d)
        self.distance_lock.release()
        self.pub_distance_traveled.publish(self.distance_traveled)

    # velocity is in m/s
    def callback_lin_read_velocity(self, msg):
        # NOTE: the linear motor is flipped as of March 27th
        #       such that a negative velocity indicates going down, and vise versa.
        value = -1 * msg.data
        
        self.linear_velocity = value

        # t = time.time()
        # self.distance_lock.acquire()
        # velocity = msg.data
        # td = t - self.distance_last_updated
        # self.distance_traveled += (td * velocity)
        # self.distance_last_updated = t
        # self.distance_lock.release()
        # self.pub_distance_traveled.publish(self.distance_traveled)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rc = DistanceTraveled()
    rc.run()
