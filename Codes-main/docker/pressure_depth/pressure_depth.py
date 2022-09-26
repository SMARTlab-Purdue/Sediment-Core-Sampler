#!/usr/bin/python3
import time
from multiprocessing import Queue
import serial

import rospy
from std_msgs.msg import Float32, Bool

import time

class PressureDepth():
    def __init__(self):
        rospy.init_node('Pressure_depth', anonymous=True)
        self.pub_pressure_upper = rospy.Publisher('/sampler/pressure/upper/pressure', Float32, queue_size=1)
        self.pub_pressure_lower = rospy.Publisher('/sampler/pressure/lower/pressure', Float32, queue_size=1)
        self.pub_temperature_upper = rospy.Publisher('/sampler/pressure/upper/temp', Float32, queue_size=1)
        self.pub_temperature_lower = rospy.Publisher('/sampler/pressure/lower/temp', Float32, queue_size=1)
        self.pub_depth_upper = rospy.Publisher('/sampler/depth/upper', Float32, queue_size=1)
        self.pub_depth_lower = rospy.Publisher('/sampler/depth/lower', Float32, queue_size=1)
        self.pub_depth_rho = rospy.Publisher('/sampler/depth/rho', Float32, queue_size=1)

        self.sub_depth_calibration = rospy.Subscriber('/sampler/depth/cmd/calibration', Bool, self.callback_depth_calibration, queue_size=1)

        self.do_calibration_queue = Queue(1)
        self.pressure_upper_queue = Queue(10)
        self.pressure_lower_queue = Queue(10)
        self.rhoSI = 1000.  # distilled water density kg/m^3
        self.grav = 9.80665 # accerleration due to gravity
        self.h1_h2 = 0.6    # fixed distance between the two sensor (m)

    # Do calibration
    def callback_depth_calibration(self, msg):
        if msg.data == True:
            self.do_calibration_queue.put(1)

    def calibrate(self, upper_values, lower_values):
        new_rhoSI = 0.
        for h1value, h2value in zip(upper_values, lower_values):
            new_rhoSI += -(h1value-h2value)/(grav*h1_h2)
        return new_rhoSI / len(upper_values)

    #convert pressure in mBar to Pascal
    def mBar2Pa(self, p_mBar):
        return p_mBar*100;

    #calculates depth (m)
    def depth(self, p, rhoSI):
        return (p-101325)/(rhoSI*self.grav)  #minus stadard atm pressure at sea level

    def run(self):
        rate = rospy.Rate(10)
        with serial.Serial("/dev/wabash_pressure_device", 9600, timeout=3) as ser:
            while not rospy.is_shutdown():
                # Input format should be "up 996.60 ut 25.83 lp 997.00 lt 25.67"
                try:
                    raw = ser.readline().decode().strip()
                except:
                    break
                split_raw = raw.split()
                if len(split_raw) != 8:
                    print("Failed to parse %s", raw)
                    continue

                # Parse the serial input
                pressure_upper = float(split_raw[1])
                temp_upper = float(split_raw[3])
                pressure_lower = float(split_raw[5])
                temp_lower = float(split_raw[7])

                if not self.do_calibration_queue.empty():
                    if self.pressure_upper_queue.full():
                        upper = []
                        lower = []
                        while True:
                            if self.pressure_upper_queue.empty() or self.pressure_lower_queue.empty():
                                break
                            upper.append(self.pressure_upper_queue.get())
                            lower.append(self.pressure_lower_queue.get())
                        self.rhoSI = self.calibrate(upper, lower)
                        print("Calibration is done")
                        self.do_calibration_queue.get()
                    else:
                        print("Calibration is in progress...")
                        if self.pressure_upper_queue.full():
                            self.pressure_upper_queue.get()
                        self.pressure_upper_queue.put(pressure_upper)
                        
                        if self.pressure_lower_queue.full():
                            self.pressure_lower_queue.get()
                        self.pressure_lower_queue.put(pressure_lower)

                # Normal operation (calculating depth)
                depth_upper = self.depth(self.mBar2Pa(pressure_upper), self.rhoSI)
                self.pub_depth_upper.publish(depth_upper)
                depth_lower = self.depth(self.mBar2Pa(pressure_lower), self.rhoSI)
                self.pub_depth_lower.publish(depth_lower)
                self.pub_temperature_upper.publish(temp_upper)
                self.pub_temperature_lower.publish(temp_lower)
                self.pub_depth_rho.publish(self.rhoSI)
                self.pub_pressure_upper.publish(pressure_upper)
                self.pub_pressure_lower.publish(pressure_lower)
                rate.sleep()
        return 1


if __name__ == '__main__':
    rc = PressureDepth()
    exit(rc.run())
