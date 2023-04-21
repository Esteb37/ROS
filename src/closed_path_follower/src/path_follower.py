#!/usr/bin/env python

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist, Pose
import numpy as np
import rospy


class PathFollower():

    TRACK_WIDTH = 0.19
    WHEEL_RADIUS = 0.05

    DISTANCE_TOLERANCE = 0.1

    def __init__(self):

        self.pose = Pose()
        self.goal = Pose()

        self.linear_vel = 0
        self.angular_vel = 0
        self.wl = 0
        self.wr = 0

        self.init_time = rospy.get_time()

        # Setup ROS node
        rospy.on_shutdown(self.cleanup)
        self.rate = rospy.Rate(50)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pose_pub = rospy.Publisher(
            'robot_pose', Pose, queue_size=1)
        self.reached_pub = rospy.Publisher(
            'reached_goal', Bool, queue_size=1)

        # Subscribers
        rospy.Subscriber("/wl", Float32, self.wl_cb)
        rospy.Subscriber("/wr", Float32, self.wr_cb)
        rospy.Subscriber("/linear_vel", Float32, self.linear_vel_cb)
        rospy.Subscriber("/angular_vel", Float32, self.angular_vel_cb)
        rospy.Subscriber("/goal", Pose, self.goal_cb)

        while rospy.get_time() == 0:
            print("No simulated time received")

        print("Running...")
        while not rospy.is_shutdown():
            self.follow_path()
            self.rate.sleep()

    def follow_path(self):
        """
            Updates the robot position, checks if it has reached the goal. If not, it sends the velocities sent by the PID controlers.
        """

        self.update_position()

        dx = self.goal.position.x - self.pose.position.x
        dy = self.goal.position.y - self.pose.position.y

        distance = np.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dy, dx) - self.pose.orientation.z

        if distance < self.DISTANCE_TOLERANCE:
            self.publish_vel(0, 0)
            self.reached_pub.publish(True)
        else:
            self.publish_vel(distance*0.14, angle*2.3)
            self.reached_pub.publish(False)

    def update_position(self):
        """
            Updates the current heading and position of the robot based on the current wheel velocities.
        """

        dt = rospy.get_time()-self.init_time

        self.linear_vel = self.WHEEL_RADIUS * (self.wr+self.wl)/2
        self.angular_vel = self.WHEEL_RADIUS * \
            (self.wr-self.wl)/self.TRACK_WIDTH

        theta = self.angular_vel * dt

        self.pose.orientation.z += np.arctan2(
            np.sin(theta), np.cos(theta))

        self.pose.position.x += self.linear_vel * \
            dt * np.cos(self.pose.orientation.z)
        self.pose.position.y += self.linear_vel * \
            dt * np.sin(self.pose.orientation.z)

        self.init_time = rospy.get_time()

        self.pose_pub.publish(self.pose)

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

    def linear_vel_cb(self, msg):
        self.linear_vel = msg.data

    def angular_vel_cb(self, msg):
        self.angular_vel = msg.data

    def goal_cb(self, msg):
        self.goal = msg


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":

    rospy.init_node("path_follower", anonymous=True)
    PathFollower()
