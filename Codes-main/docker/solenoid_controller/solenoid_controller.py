#!/usr/bin/python3
import time
from multiprocessing import Queue

import rospy
from std_msgs.msg import Float32, Byte

import time

class SolenoidController():
    def __init__(self):
        rospy.init_node('Sampler_Solenoid_Controller', anonymous=True)
        self.interval_queue = Queue()
        self.sub_solenoid = rospy.Subscriber('/sampler/cmd/solenoid', Float32, self.callback, queue_size=1)
        self.pub_solenoid = rospy.Publisher('/sampler/solenoid', Byte, queue_size=1)

    def callback(self, msg):
        self.interval_queue.put(msg.data)

    def run(self):
        solenoid_interval = 0.
        alternate_values = [0x00, 0x01]
        index = 0
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if not self.interval_queue.empty():
                solenoid_interval = self.interval_queue.get()
                if solenoid_interval < 0.:
                    self.pub_solenoid.publish(0x00)

            if solenoid_interval > 0.:
                index = (index + 1) % 2
                self.pub_solenoid.publish(alternate_values[index])
                time.sleep(solenoid_interval)
            rate.sleep()


if __name__ == '__main__':
    rc = SolenoidController()
    rc.run()
