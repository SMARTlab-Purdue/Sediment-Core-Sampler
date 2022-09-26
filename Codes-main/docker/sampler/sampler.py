#!/usr/bin/python3
import time
import threading
from multiprocessing import Process, Queue

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Byte, Bool, String, Int32MultiArray, Float32

import time

class RotaryController(Process):
    def __init__(self, kill_queue, rotary_input):
        # rospy.init_node('Sampler_Rotary_Controller', anonymous=True)
        self.terminate_queue = kill_queue
        self.rotary_input = rotary_input
        self.angle = 0
        self.sub_rot_position = rospy.Subscriber('/sampler/motor/rotary/read_position', Int32, self.callback_position, queue_size=1)
        self.pub_rot_position = rospy.Publisher('/sampler/motor/rotary/write_position', Int32, queue_size=1)
        self.current_position = 2048

    def callback_position(self, msg):
        self.current_position = msg.data

    def run(self):
        while True:
            if not self.terminate_queue.empty():
                break
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
            time.sleep(0.1)   

class SamplerController():
    def __init__(self):
        rospy.init_node('Sampler_Controller', anonymous=True)
        self.sub_init = rospy.Subscriber('/sampler/cmd/init', Bool, self.callback_init, queue_size=1)
        self.sub_dig = rospy.Subscriber('/sampler/cmd/dig', Int32MultiArray, self.callback_dig, queue_size=1)
        self.sub_halt = rospy.Subscriber('/sampler/cmd/halt', Bool, self.callback_halt, queue_size=1)
        self.halt = False

        # self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1)
        self.sub_lswitch = rospy.Subscriber('/sampler/switch/lower', Byte, self.ls_callback, queue_size=1)
        self.lswitch_pressed = False
        self.sub_uswitch = rospy.Subscriber('/sampler/switch/upper', Byte, self.us_callback, queue_size=1)
        self.uswitch_pressed = False

        # calculate distance traveled
        self.distance_lock = threading.Lock()
        self.sub_reset_distance_traveled = rospy.Subscriber('/sampler/motor/linear/distance_traveled/cmd/reset', Bool, self.callback_reset_distance_traveled, queue_size=1)
        self.pub_distance_traveled = rospy.Publisher('/sampler/motor/linear/distance_traveled', Float32, queue_size=1)
        self.sub_lin_velocity = rospy.Subscriber('/sampler/motor/linear/read_velocity', Float32, self.callback_lin_read_velocity, queue_size=1)
        self.linear_velocity = 0.
        self.distance_traveled = 0.
        self.distance_last_updated = -1

        self.sub_rot_position = rospy.Subscriber('/sampler/motor/rotary/read_position', Int32, self.callback_rot_read_position, queue_size=1)
        self.rot_position = 0

        self.pub_error = rospy.Publisher('/sampler/controller/error', String, queue_size=10)
        self.pub_lin_speed = rospy.Publisher('/sampler/motor/linear/write_speed', Int32, queue_size=1)
        
        self.sol_pub = rospy.Publisher('/sampler/solenoid', Byte, queue_size=1)

        # self.status_pub = rospy.Publisher('/sediment_sampler', Int32, queue_size=10)
        # self.status = 0
        # self.flip = False

    def __enter__(self):
        self.rot_controller_kill = Queue()
        self.rot_controller_signal = Queue()
        self.rot_controller = RotaryController(self.rot_controller_kill, self.rot_controller_signal)
        self.rot_controller.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.rot_controller_kill.put(1)
        self.rot_controller.join()

    def callback_halt(self, msg):
        self.halt = msg.data

    def callback_init(self, msg):
        if msg.data == False:
            return
        # STEP1: Sampler moving upward until it touches the limit switch
        self.pub_lin_speed.publish(-1000)
        success = False
        for i in range(100):
            if self.uswitch_pressed == True:
                success = True
                break
            time.sleep(1)
        if success is False:
            self.pub_error.publish("Failed to init: sampler could not move to 0 position")
            return

        # STEP2: Wait until the motor stops (it moves by the OpenCR logic)
        success = False
        for i in range(100):
            if self.linear_velocity == 0.:
                success = True
                break
        if success is False:
            sulf.pub_error.publish("Failed to init: sampler never stops")
            return

        # STEP3: Sampler moving downward for 5 mm
        self.pub_lin_speed.publish(1000)
        time.sleep(4)
        self.pub_lin_speed.publish(0)

    def callback_fallback(self, msg):
        pass

    def callback_rot_read_position(self, msg):
        self.rot_position = msg.data

    def callback_dig(self, msg):
        if len(msg.data) < 2:
            self.pub_error.publish("Failed to dig: not enough input {}".format(msg.data))
            return

        linear_speed = msg.data[0]
        if linear_speed < 0:
            self.pub_error.publish("Failed to dig: linear speed must be positive {}".format(linear_speed))
            return

        rotate_angle = msg.data[1]
        if rotate_angle > 4000:
            self.pub_error.publish("Failed to dig: rotary angle must be within 4000, {}".format(rotate_angle))
            return

        self.pub_rot_position.publish(2048)
        for i in range(10):
            if self.rot_position == 2048:
                break
            time.sleep(1)
        # if self.rot_position != 0:
        #     self.pub_error.publish("Failed to dig: could not position rotary to 0")
        #     return

        # start digging
        self.pub_lin_speed.publish(linear_speed)
        print("digging...")
        delta_position = rotate_angle
        target_position = 2048 + delta_position
        self.pub_rot_position.publish(target_position)
        time.sleep(1)
        while self.halt == False and self.lswitch_pressed == False:
            if abs(target_position - self.rot_position) < 30:
                delta_position = -1 * delta_position
                target_position = 2048 + delta_position
            self.pub_rot_position.publish(target_position)
            print(target_position)
            time.sleep(0.1)

    # reset distance traveled
    def callback_reset_distance_traveled(self, msg):
        self.distance_lock.acquire()
        self.distance_traveled = 0.
        self.distance_last_updated = -1
        self.distance_lock.release()

    # velocity is in m/s
    def callback_lin_read_velocity(self, msg):
        pass
        # self.linear_velocity = msg.data

        # t = msg.header.stamp.to_time()
        # self.distance_lock.acquire()
        # if self.distance_last_updated < 0:
        #     self.distance_last_updated = t
        #     return
        # velocity = msg.data
        # td = t - self.distance_last_updated
        # self.distance_traveled += (td * velocity)
        # self.distance_last_updated = t
        # self.distance_lock.release()
        # self.pub_distance_traveled.publish(self.distance_traveled)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
			# self.status_pub.publish(self.status)
			# if self.status == 0:
			# 	self.lin_motor_pub.publish(0)
			# if self.status == 1:
			# 	self.lin_motor_pub.publish(1000)
			# 	if self.flip:
			# 		self.rot_motor_pub.publish(4000)
			# 	else:
			# 		self.rot_motor_pub.publish(0)
			# 	time.sleep(0.1)
			# 	self.flip = not self.flip
            #
			# if self.status == 2:
			# 	self.lin_motor_pub.publish(0)
            #
			# if self.status == 3:
			# 	self.lin_motor_pub.publish(-1000)
            rate.sleep()

    def ls_callback(self, data):
        if data.data == 1:
            self.lswitch_pressed = True
        else:
            self.lswitch_pressed = False

    def us_callback(self, data):
        if data.data == 1:
            self.uswitch_pressed = True
        else:
            self.uswitch_pressed = False

	# def joy_callback(self, data):
	# 	'''
	# 	Summary:
	# 	Call back for joy updates from joy node and lower and raise the anchor.
    #
	# 	Parameters:
	# 	data (Joy): Button and axis readings from the Joystick
    #
	# 	Returns:
	# 	None
	# 	'''
	# 	if (data.buttons[2] == 1.0 and data.buttons[4] == 1.0):
	# 		self.status += 1
	# 		self.status %= 4
	# 		time.sleep(1.0)
    #
	# 	if (data.buttons[3] == 1.0 and data.buttons[4] == 1.0):
	# 		self.sol_pub.publish(1)



if __name__ == '__main__':
    with SamplerController() as sc:
        sc.spin()
