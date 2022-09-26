#!/usr/bin/python3
import time
from multiprocessing import Queue

import rospy
from std_msgs.msg import Int32

from simple_pid import PID

class LinearController():
    def __init__(self):
        rospy.init_node('Sampler_Linear_Controller', anonymous=True)
        self.rotary_input = Queue()
        self.angle = 0
        self.sub_rotary_drive = rospy.Subscriber('/sampler/cmd/rotary', Int32, self.callback_drive, queue_size=1)
        self.sub_rot_position = rospy.Subscriber('/sampler/motor/rotary/read_position', Int32, self.callback_position, queue_size=1)
        self.pub_rot_position = rospy.Publisher('/sampler/motor/rotary/write_position', Int32, queue_size=1)
        self.current_position = 2048

    def callback_drive(self, msg):
        self.rotary_input.put(msg.data)

    def callback_position(self, msg):
        self.current_position = msg.data

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if not self.rotary_input.empty():
                self.angle = self.rotary_input.get()
                if self.angle > 2047:
                    self.angle = 2047
                self.target_pos = [2048 - self.angle, 2048 + self.angle]
                self.target_index = 0
                self.pub_rot_position.publish(self.target_pos[self.target_index])

            if self.angle > 0:
                if abs(self.target_pos[self.target_index] - self.current_position) < 30:
                    self.target_index = (self.target_index + 1) % 2
                    self.pub_rot_position.publish(self.target_pos[self.target_index])
                    print("changed to {}".format(self.target_pos[self.target_index]))
            rate.sleep()


if __name__ == '__main__':
    rc = RotaryController()
    rc.run()
