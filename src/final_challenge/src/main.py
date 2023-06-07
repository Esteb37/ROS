#!/usr/bin/env python

from std_msgs.msg import Float32, Int32, String, Bool
from geometry_msgs.msg import Twist, Pose
import numpy as np
import rospy
from time import time
import sys

class Robot():

    TRACK_WIDTH = 0.19
    WHEEL_RADIUS = 0.05

    def __init__(self):

        self.system = sys.argv[1]

        self.LINEAR_VELOCITY = rospy.get_param("/"+self.system+"_linear_velocity", 0.5)

        self.linear_vel = 0
        self.angular_vel = 0

        self.line_angular_vel = 0
        self.turn_angular_vel = 0
        self.crossroad = -1
        self.prev_crossroad = -1
        self.distance_time = 0
        self.traffic_sign = "none"

        self.wl = 0
        self.wr = 0
        self.heading = 0

        self.traffic_light_status = "none"
        self.is_stopped = False
        self.turning = False
        self.driving = False
        self.crossing = False

        self.init_time = rospy.get_time()

        self.setup_node()

        print("Running...")

                    # Turn left
        actions = [[("drive",0.3),("turn", np.pi/2), ("drive",0.5)],

                    # Cross road
                    [("drive",0.5)]]

        curr_command = 0
        curr_action = 0

        while not rospy.is_shutdown():
            if self.crossroad > 150 and not self.crossing:
                self.crossing = True

            if self.crossing:
                print("crossing")
                commands = actions[curr_action]
                command, value = commands[curr_command]

                if command == "drive":
                    if self.drive(value):
                        curr_command += 1
                elif command == "turn":
                    if self.turn(value):
                        curr_command += 1

                if curr_command >= len(commands):
                    self.crossing = False
                    curr_command = 0
                    curr_action += 1

            else:
                print("following")
                self.follow_line()

            self.crossing_pub.publish(self.crossing)
            self.rate.sleep()



    def setup_node(self):
        rospy.on_shutdown(self.cleanup)
        self.rate = rospy.Rate(50)
        self.setup_publishers()
        self.setup_subscribers()

        # Setup ROS node
        print("Waiting for time to be set...")
        while rospy.get_time() == 0:
            pass

    def setup_publishers(self):
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10)
        self.turn_error_pub = rospy.Publisher(
            '/turn_error', Float32, queue_size=10)
        self.crossing_pub = rospy.Publisher(
            "/crossing", Bool, queue_size = 10)


    def setup_subscribers(self):
        rospy.Subscriber("/wl", Float32, self.wl_cb)
        rospy.Subscriber("/wr", Float32, self.wr_cb)
        rospy.Subscriber("/traffic_light", String, self.traffic_light_cb)
        rospy.Subscriber("/line_angular_vel", Float32,
                         self.line_angular_vel_cb)
        rospy.Subscriber("/turn_angular_vel", Float32,
                         self.turn_angular_vel_cb)
        rospy.Subscriber("/crossroad", Int32, self.crossroad_cb)
        rospy.Subscriber("/sign", String, self.traffic_sign_cb)
        # rospy.Subscriber("/linear_vel", Float32, self.linear_vel_cb)

    def sign_exist(self):

        if self.traffic_sign != "none":

            if self.sign == "left":
                self.turn(self, -np.pi/2)

            elif self.sign == "right":
                self.turn(self, np.pi/2)

            elif self.sign == "stop":
                self.publish_vel(0,0)

            elif self.sign == "move":
                self.publish_vel(self.LINEAR_VELOCITY, self.line_angular_vel)

            elif self.sign == "construction":
                self.publish_vel(self.LINEAR_VELOCITY/2, self.line_angular_vel)

            else:
                self.publish_vel(self.LINEAR_VELOCITY, self.line_angular_vel)

        return self.traffic_sign


    def follow_line(self):
        if self.is_stopped:

            self.publish_vel(0, 0)

            if self.traffic_light_status == "green":
                self.publish_vel(self.LINEAR_VELOCITY, self.line_angular_vel)
                self.is_stopped = False
                self.sign_exist()

        else:
            if self.traffic_light_status == "red":
                self.publish_vel(0, 0)
                self.is_stopped = True

            elif self.traffic_light_status == "yellow":
                self.publish_vel(self.LINEAR_VELOCITY/2, self.line_angular_vel)
                self.sign_exist()

            else:
                self.publish_vel(self.LINEAR_VELOCITY, self.line_angular_vel)

    def drive(self, target):

        if not self.driving:
            self.distance_time = rospy.get_time()
            self.driving = True

        self.publish_vel(self.LINEAR_VELOCITY, 0)
        vel = self.WHEEL_RADIUS * (self.wr+self.wl)/2
        distance = vel * (rospy.get_time() - self.distance_time)

        if distance > target:
            self.driving = False
            return True

        return False

    def update_heading(self):
        """
            Updates the current heading of the robot based on the current wheel velocities.
        """

        dt = rospy.get_time()-self.init_time

        angular_vel = self.WHEEL_RADIUS * \
            (self.wr-self.wl)/self.TRACK_WIDTH

        theta = angular_vel * dt

        self.heading += np.arctan2(
            np.sin(theta), np.cos(theta))

        self.init_time = rospy.get_time()

    def turn(self, angle):
        """
            Turns the robot by a given angle.
        """
        if not self.turning:
            self.heading = 0
            self.turning = True

        self.update_heading()
        self.turn_error_pub.publish(angle - self.heading)
        self.publish_vel(0, self.turn_angular_vel)

        if np.abs(angle - self.heading) < 0.1:
            self.turning = False
            return True

        return False

    def publish_vel(self, linear, angular):
        """
            Sends a velocity command to the robot through the cmd_vel topic.
        """
        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = np.clip(angular, -5, 5)
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

    def line_angular_vel_cb(self, msg):
        self.line_angular_vel = msg.data

    def turn_angular_vel_cb(self, msg):
        self.turn_angular_vel = msg.data

    def crossroad_cb(self, msg):
        self.crossroad = msg.data

    def traffic_sign_cb(self, msg):
        self.traffic_sign = msg.data


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    rospy.init_node("final_challenge", anonymous=True)
    Robot()
