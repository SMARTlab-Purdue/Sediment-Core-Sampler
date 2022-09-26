#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

def webcam_pub():
    pub = rospy.Publisher('webcam/image_raw', Image, queue_size=1)
    rospy.init_node('stream_camera', anonymous=True)
    rate = rospy.Rate(60) # 60hz

    cam = cv2.VideoCapture(0)
    bridge = CvBridge()

    if not cam.isOpened():
         sys.stdout.write("Webcam is not available")
         return -1

    while not rospy.is_shutdown():
        ret, frame = cam.read()
        
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        if ret:
            pass
        
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
	try:
		webcam_pub()
	except rospy.ROSInterruptException:
		pass
