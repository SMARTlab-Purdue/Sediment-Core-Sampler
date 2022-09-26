import rospy
import time
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('test/int', Int32, queue_size=1)
    pub2 = rospy.Publisher('test/nint', Int32, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(4) # 10hz
    i = 0
    while not rospy.is_shutdown():

        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        pub.publish(i)
        pub2.publish(-1 * i)
        i += 1
        if i > 10:
            i = 0
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
