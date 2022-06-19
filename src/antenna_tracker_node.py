#!/usr/bin/env python

import rospy
from math import pi, radians, degrees
from geographiclib.geodesic import Geodesic
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from threading import Lock
from antenna_tracker.srv import SetCoordinates

class AntennaTracker():
    def __init__(self):
        rospy.init_node("antenna_tracker", anonymous=True)
        self.mutex = Lock()
        self.base_station_coordinates = (
            rospy.get_param("~latitude", 0),
            rospy.get_param("~longitude", 0)
        )
        self.s = rospy.Service('antenna_tracker/set_coordinates', SetCoordinates, self.handle_set_coordinates)
        self.orientation_pub = rospy.Publisher("antenna_goal_orientation", Float32, queue_size=1)
        self.gps_sub = rospy.Subscriber("fix", NavSatFix, callback=self.gpsCB)

    def gpsCB(self, msg):
        self.mutex.acquire()
        try:
            yaw = calculate_bearing(self.base_station_coordinates, (msg.latitude, msg.longitude))
        finally:
            self.mutex.release()
        self.orientation_pub.publish(Float32(yaw))
        print("Orientation: %.2f" % degrees(yaw))

    def handle_set_coordinates(self, req):
        rospy.loginfo("Setting base station coordinates to: %f, %f" % (req.latitude, req.longitude))
        self.mutex.acquire()
        try:
            self.base_station_coordinates = (req.latitude, req.longitude)
        finally:
            self.mutex.release()
        return True


def wrap_angle_pi(angle):
    # Wraps an angle in radians to -pi : pi
    return (angle + pi) % (2 * pi) - pi


def calculate_bearing(p1, p2):
    # Returns the bearing from point 1 to point 2 (lat/long). 
    # by default, the geodesic azimuth value is in degrees, from -pi to pi, with 0 pointing North.
    # Positive is clockwise and negative is counter-clockwise
    # To match the ROS convention, units have to be in radians, the zero should point east and
    # positive should be counter-clockwise
    return wrap_angle_pi(-radians(Geodesic.WGS84.Inverse(p1[0], p1[1], p2[0], p2[1])['azi1']) + pi/2)


if __name__ == '__main__':
    gps_heading = AntennaTracker()
    rospy.loginfo("antenna_tracker ready")
    rospy.spin()