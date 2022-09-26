import time
from multiprocessing import Queue, Lock

from serial import Serial

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class Controller(object):
    def __init__(self):
        rospy.init_node('usv_motorcontroller', anonymous=True)

        self.sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.twistCallback)
        self.sub_anchor_left_up = rospy.Subscriber("/usv/anchor/left/motor/up", Bool, self.callback_motor, callback_args=("an l 255"))
        self.sub_anchor_left_stop = rospy.Subscriber("/usv/anchor/left/motor/stop", Bool, self.callback_motor, callback_args=("an l 127"))
        self.sub_anchor_left_down = rospy.Subscriber("/usv/anchor/left/motor/down", Bool, self.callback_motor, callback_args=("an l 0"))
        
        self.sub_anchor_right_up = rospy.Subscriber("/usv/anchor/right/motor/up", Bool, self.callback_motor, callback_args=("an r 255"))
        self.sub_anchor_right_stop = rospy.Subscriber("/usv/anchor/right/motor/stop", Bool, self.callback_motor, callback_args=("an r 127"))
        self.sub_anchor_right_down = rospy.Subscriber("/usv/anchor/right/motor/down", Bool, self.callback_motor, callback_args=("an r 0"))
        
        self.sub_sampler_up = rospy.Subscriber("/usv/sampler/motor/up", Bool, self.callback_motor, callback_args=("sa r 255"))
        self.sub_sampler_stop = rospy.Subscriber("/usv/sampler/motor/stop", Bool, self.callback_motor, callback_args=("sa r 127"))
        self.sub_sampler_down = rospy.Subscriber("/usv/sampler/motor/down", Bool, self.callback_motor, callback_args=("sa r 0"))

        self.sub_right_clutch = rospy.Subscriber("/usv/anchor/right/clutch", Bool, self.callback_clutch, callback_args=("clr"), queue_size=1)
        self.sub_left_clutch = rospy.Subscriber("/usv/anchor/left/clutch", Bool, self.callback_clutch, callback_args=("cll"), queue_size=1)
        self.sub_sampler_clutch = rospy.Subscriber("/usv/sampler/clutch", Bool, self.callback_clutch, callback_args=("cl"), queue_size=1)
        
        self.input_commands = Queue()
        self.mutex = Lock()

    def _convert_vel_to_command(self, vel):
        # vel is expected to be ranged in [-1., 1.]
        lower = (0., 126.)
        neutral = 127.
        upper =(128., 255.)
        c = 0.
        if vel == 0.:
            return neutral
        elif vel > 0.:
            c = vel * (upper[1] - upper[0]) + upper[0]
            if c > upper[1]:
                c = upper[1]
        else:
            c = (1. - abs(vel)) * (lower[1] - lower[0]) + lower[0]
            if c > lower[1]:
                c = lower[1]
        return int(c)

    def twistCallback(self,msg):
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.w = 0.5
        self.left = (1.5 * self.dx - self.dr * self.w) / 2.0
        self.right = (1.5 * self.dx + self.dr * self.w) / 2.0

        rospy.loginfo("Right Speed: " + str(self.right) + " Left Speed: " + str(self.left))

        # # Motor 1 - Right Motor
        motor_1 = self._convert_vel_to_command(self.right)

        # Motor 2 - Left Motor
        motor_2 = self._convert_vel_to_command(self.left)
        
        msg2 = "th l " + str(motor_2)
        msg1 = "th r " + str(motor_1)
        self.access_to_queue(msg2, isGet=False)
        self.access_to_queue(msg1, isGet=False)
        rospy.loginfo("Right Cmd: " + msg1 + " Left Cmd: " + msg2)


    def callback_motor(self, msg, topic):
        if msg.data == True:
            self.access_to_queue(topic, isGet=False)

    def callback_clutch(self, msg, topic):
        if msg.data:
            self.access_to_queue(' '.join([topic, 'on']), isGet=False)
        else:
            self.access_to_queue(' '.join([topic, 'off']), isGet=False)

    def access_to_queue(self, msg=None, isGet=True):
        self.mutex.acquire()
        try:
            if isGet:
                if self.input_commands.empty():
                    return None
                else:
                    return self.input_commands.get()
            else:
                self.input_commands.put(msg)
        finally:
            self.mutex.release()

    def run(self):
        r = rospy.Rate(10)
        print("Controller started")
        with Serial("/dev/wabash_motor_driver", 115200) as motor:
            motor.write(b'init\r\n')
            while not rospy.is_shutdown():
                cmd = self.access_to_queue(None, isGet=True)
                if cmd != None:
                    cmd += "\r\n"
                    print(cmd)
                    motor.write(cmd.encode())
                r.sleep()


if __name__ == '__main__':
    usv_controller = Controller()
    usv_controller.run()
