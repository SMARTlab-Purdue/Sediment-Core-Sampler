import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, Bool, Float32


class AutoPilot():
    def __init__(self):
        rospy.init_node('wabash_autopilot', anonymous=True)
        self.sub_gps = rospy.Subscriber('/usv/gps', NavSatFix, self.gps_callback)
        self.sub_head = rospy.Subscriber('/usv/heading', Int32, self.head_callback)
        self.sub_dest = rospy.Subscriber('/usv/autopilot/destination', NavSatFix, self.dest_callback)
        self.sub_kill_switch = rospy.Subscriber('/usv/autopilot/cmd/kill', Bool, self.kill_callback)
        self.sub_cmd_engage = rospy.Subscriber('/usv/autopilot/cmd/engage', Bool, self.engage_callback)

        self.pub_engaged = rospy.Publisher('/usv/autopilot/indicator/engaged', Bool, queue_size=1)
        self.pub_distance_to_dest = rospy.Publisher('/usv/autopilot/indicator/distance_to_dest', Float32, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.engaged = False
        self.close_enough = 5  # autopilot ceased when destination within 5 m
        self.current_lat = None
        self.current_lon = None
        self.current_angle = None
        self.dest_lat = None
        self.dest_lon = None
        self.theta_e_old = 0

        self.kdp = rospy.get_param('~Kdp', 0.1)
        self.kap = rospy.get_param('~Kap', 0.4)
        self.kad = rospy.get_param('~Kad', 0.0)

    def engage_callback(self, engage_msg):
        self.engaged = engage_msg.data

    def kill_callback(self, data):
        self.engaged = False
        rospy.loginfo('Kill switch pressed. Autopilot ceased.')

    def gps_callback(self, navsatfix):
        self.current_lat = navsatfix.latitude
        self.current_lon = navsatfix.longitude

    def head_callback(self, angle):
        self.current_angle = angle.data

    def dest_callback(self, navsatfix):
        self.dest_lat = navsatfix.latitude
        self.dest_lon = navsatfix.longitude
        rospy.loginfo('New destination received: %f, %f', self.dest_lat, self.dest_lon)

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
        distance in m
        '''
        lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        km = 6367 * c
        return km * 1000 # in m

    def target_angle(self, lat1, lon1, lat2, lon2):
        '''
        Summary:
        Computes the angle between two GPS coordinates.

        Parameters:
        lon1 - longitude point 1
        lat1 - latitude point 1
        lon2 - longitude point 2
        lat2 - latitude point 2

        Returns:
        degree
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

    def _check_runnable(self):
        if self.engaged is False:
            return False, 'autopilot not engaged'
        if self.dest_lat is None:
            return False, 'No destination is set'
        if self.current_lat is None:
            return False, 'No current GPS received'
        return True, ''

    def run(self):
        r = rospy.Rate(1)
        rospy.loginfo("Autopilot started")
        while not rospy.is_shutdown():
            # publish the status of autopilot
            self.pub_engaged.publish(self.engaged)

            ret, msg = self._check_runnable()
            if ret is False:
                rospy.loginfo("Autopilot not runnable: %s", msg)
            else:
                d = self.haversine(self.dest_lat, self.dest_lon, self.current_lat, self.current_lon)
                rospy.loginfo('Distance to destination: %f m', d)
                if d <= self.close_enough:
                    rospy.loginfo('Destination (%f, %f) within %f m. Stopping the vehicle...', self.dest_lat, self.dest_lon, d)
                    t = Twist()
                    t.linear.x = 0.
                    t.angular.z = 0.
                    self.pub_cmd_vel.publish(t)
                    rospy.loginfo('Closing autopilot...')
                    self.dest_lat = None
                    self.engaged = False
                else:
                    a = self.target_angle(self.dest_lat, self.dest_lon, self.current_lat, self.current_lon)
                    rospy.loginfo('Angle to destination: %f degree', a)
                    theta_e = int(a) - self.current_angle + 180 # +180 to revert the direction
                    while (theta_e > 180):
                        theta_e -= 360
                    while (theta_e < -180):
                        theta_e += 360
                    rospy.loginfo('Angle difference: %f', theta_e)
                    linSpd = self.kdp * d
                    rotSpd = self.kap * theta_e + self.kad * (theta_e - self.theta_e_old)
                    self.theta_e_old = theta_e

                    rotSpd = math.pi * rotSpd / 180
                    rospy.loginfo('Linear speed %f, Rotation speed %f', linSpd, rotSpd)
                    t = Twist()
                    t.linear.x = linSpd
                    t.angular.z = rotSpd

                    self.pub_distance_to_dest.publish(d)
                    self.pub_cmd_vel.publish(t)
            r.sleep()


if __name__ == '__main__':
    usv_autopilot = AutoPilot()
    usv_autopilot.run()