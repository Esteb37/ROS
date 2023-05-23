#!/usr/bin/env python

from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist, Pose
import numpy as np
import rospy


class PathFollower():

    TRACK_WIDTH = 0.19
    WHEEL_RADIUS = 0.05

    DISTANCE_TOLERANCE = 0.5

    def __init__(self):

        self.linear_vel = 0
        self.angular_vel = 0
        self.wl = 0
        self.wr = 0
        self.traffic_light_status = "none"
        self.is_stopped = True
        self.line_centroid = 0

        self.init_time = rospy.get_time()

        # Setup ROS node
        rospy.on_shutdown(self.cleanup)
        self.rate = rospy.Rate(50)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher(
            'cmd_vel', Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber("/wl", Float32, self.wl_cb)
        rospy.Subscriber("/wr", Float32, self.wr_cb)
        rospy.Subscriber("/line_centroid", Float32, self.line_centroid_cb)
        rospy.Subscriber("/traffic_light", String, self.traffic_light_cb)

        print("Waiting for time to be set...")
        while rospy.get_time() == 0:
            pass

        print("Running...")

        while not rospy.is_shutdown():
            self.follow_path()
            self.rate.sleep()

    def follow_line(self):

        # TODO Drive robot forward and keep the centroid at the center of the camera

        if self.is_stopped:

            self.publish_vel(0, 0)

            if self.traffic_light_status == "green":
                self.publish_vel(self.linear_vel, self.angular_vel)
                self.is_stopped = False

        else:
            if self.traffic_light_status == "red":
                self.is_stopped = True

            elif self.traffic_light_status == "yellow":
                self.publish_vel(self.linear_vel * 0.5, self.angular_vel)

            else:
                self.publish_vel(self.linear_vel, self.angular_vel)

    def publish_vel(self, linear, angular):
        """
            Sends a velocity command to the robot through the cmd_vel topic.
        """
        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular
        self.cmd_vel_pub.publish(vel)

    def cleanup(self):
        """
            Stops the robot when the program is interrupted.
        """
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)

        print("My battery is low and it's getting dark")

    def wl_cb(self, wl):
        self.wl = wl.data

    def wr_cb(self, wr):
        self.wr = wr.data

    def traffic_light_cb(self, msg):
        self.traffic_light_status = msg.data

    def light_centroid_cb(self, msg):
        self.line_centroid = msg.data


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":

    args = rospy.myargv()[1:]

    rospy.init_node("path_follower", anonymous=True)
    PathFollower()
