#!/usr/bin/env python

from std_msgs.msg import Float32, Int32, String, Bool
from geometry_msgs.msg import Twist
from final_challenge.msg import detected_object
import numpy as np
import rospy
import sys

class Robot():

    TRACK_WIDTH = 0.19
    WHEEL_RADIUS = 0.05

    # Simulation or Jetson
    SYSTEM = sys.argv[1]


    FULL_VELOCITY = rospy.get_param("/"+SYSTEM+"_linear_velocity", 0.5)

    # Commands
    TURN_RIGHT_CMD = [("drive",0.2, FULL_VELOCITY*0.8),
                    ("turn", np.pi/2, 1),
                    ("drive",0.2, FULL_VELOCITY*0.8)]

    TURN_LEFT_CMD = [("drive",0.2, FULL_VELOCITY*0.8),
                     ("turn", -np.pi/2, 1),
                     ("drive",0.2, FULL_VELOCITY*0.8)]

    DRIVE_FORWARD_CMD = [("drive", 0.5, FULL_VELOCITY * 0.8)]


    CROSSROAD_OBEY_THRESHOLD = 150

    STOP_TIME = 10

    GIVE_WAY_TIME = 5

    ROAD_WORK_TIME = 2

    def __init__(self):

        self.yolo_started = False

        self.linear_vel = 0
        self.angular_vel = 0

        self.line_angular_vel = 0
        self.turn_angular_vel = 0

        # Crossroad detection
        self.crossroad = -1
        self.prev_crossroad = -1

        # Road detections
        self.traffic_sign = "none"
        self.last_valid_sign = "none"
        self.traffic_light = "none"

        # Odometry
        self.wl = 0
        self.wr = 0
        self.heading = 0
        self.distance_time = 0

        # State machine variables
        self.is_stopped = False
        self.stop_released = True
        self.turning = False
        self.driving = False
        self.crossing = False
        self.stopping = False
        self.stopping_time = 0
        self.giving_way = False
        self.giving_way_time = 0
        self.road_work = False
        self.road_work_time = 0

        self.init_time = rospy.get_time()

        self.setup_node()

        print("[MAIN]  Waiting for YOLO...")
        while not self.yolo_started:
            pass

        print("[MAIN]  Running...")

        self.current_action = 0

        crossing = False

        while not rospy.is_shutdown():

            self.follow_line()

            #crossing = self.check_for_crossroad(self.crossroad, crossing)

            #if crossing:
                #self.cross_road()

            #else:
                #self.check_sign()
                #self.follow_line()

            #self.crossing_pub.publish(self.crossing)
            self.rate.sleep()

    def setup_node(self):
        rospy.on_shutdown(self.cleanup)
        self.rate = rospy.Rate(50)
        self.setup_publishers()
        self.setup_subscribers()

        # Setup ROS node
        print("[MAIN]  Waiting for time to be set...")
        while rospy.get_time() == 0:
            pass

    def setup_publishers(self):
        self.cmd_vel_pub = rospy.Publisher(
            '/puzzlebot/cmd_vel', Twist, queue_size = 10)
        self.turn_error_pub = rospy.Publisher(
            '/turn_error', Float32, queue_size = 10)
        self.crossing_pub = rospy.Publisher(
            "/crossing", Bool, queue_size = 10)

    def setup_subscribers(self):
        rospy.Subscriber("/puzzlebot/wl", Float32, self.wl_cb)
        rospy.Subscriber("/puzzlebot/wr", Float32, self.wr_cb)
        rospy.Subscriber("/traffic_light", detected_object, self.traffic_light_cb)
        rospy.Subscriber("/line_angular_vel", Float32,
                         self.line_angular_vel_cb)
        rospy.Subscriber("/turn_angular_vel", Float32,
                         self.turn_angular_vel_cb)
        rospy.Subscriber("/crossroad", Int32, self.crossroad_cb)
        rospy.Subscriber("/sign", detected_object, self.traffic_sign_cb)
        rospy.Subscriber("/yolo_started", Bool, self.yolo_started_cb)
        # rospy.Subscriber("/linear_vel", Float32, self.linear_vel_cb)

    def check_for_crossroad(self, crossroad, crossing):
        return crossroad > self.CROSSROAD_OBEY_THRESHOLD and not crossing

    def cross(self):

        if self.sign == "left":
            command = self.TURN_LEFT_CMD
        elif self.sign == "right":
            command = self.TURN_RIGHT_CMD
        elif self.sign == "forward" or self.sign == "none":
            command = self.DRIVE_FORWARD_CMD

        action, target, speed = command[self.current_action]

        if action == "drive":
            if self.drive(target, speed):
                self.current_action += 1
        elif action == "turn":
            if self.turn(target, speed):
                self.current_action += 1

        if self.current_action >= len(command):
            self.crossing = False
            self.current_action = 0


    def follow_line(self):
        if self.traffic_light == "red":
            print("[MAIN] Stopped")
            self.is_stopped = True
            self.publish_vel(0, 0)

        elif self.traffic_light == "yellow":
            print("[MAIN] Yellow. Slowing.")
            self.is_stopped = False
            self.publish_vel(self.FULL_VELOCITY/2, self.line_angular_vel*2)

        elif self.traffic_light == "green" or (self.traffic_light == "none" and not self.is_stopped):
            self.is_stopped = False

            if (self.traffic_sign == "stop" or self.stopping) and self.stop_released:

                if not self.stopping:
                    print("[MAIN] Stopped started.")
                    self.stopping_time = rospy.get_time()
                self.stopping = True
                self.publish_vel(0, 0)
                print("[MAIN] Stopped.")
                if rospy.get_time() - self.stopping_time > self.STOP_TIME:
                    self.stopping = False
                    self.stop_released = False
                    print("[MAIN] Stopped finished.")

            elif self.traffic_sign == "give_way" or self.giving_way:
                self.stop_released = True
                if not self.giving_way:
                    print("[MAIN] Give way started.")
                    self.giving_way_time = rospy.get_time()
                self.giving_way = True
                self.publish_vel(self.FULL_VELOCITY/2, self.line_angular_vel)
                print("[MAIN] Giving way.")
                if rospy.get_time() - self.giving_way_time > self.GIVE_WAY_TIME:
                    print("[MAIN] Giving way finished.")
                    self.giving_way = False

            elif self.traffic_sign == "road_work" or self.road_work:
                self.stop_released = True
                if not self.road_work:
                    print("[MAIN] Road work started.")
                    self.road_work_time = rospy.get_time()

                self.publish_vel(self.FULL_VELOCITY/2, self.line_angular_vel*2)
                self.road_work = True

                print("[MAIN] Slowing for work.")
                if rospy.get_time() - self.road_work_time > self.road_work:
                    print("[MAIN] Road work ended.")
                    self.road_work = False

            else:
                print("[MAIN] Normal speed.")
                self.publish_vel(self.FULL_VELOCITY, self.line_angular_vel)


    def follow_line(self):
        self.publish_vel(self.FULL_VELOCITY, self.line_angular_vel)
        """if self.is_stopped:

            self.publish_vel(0, 0)

            if self.traffic_light_status == "green":
                self.publish_vel(self.FULL_VELOCITY, self.line_angular_vel)
                self.is_stopped = False
                self.sign_exist()

        else:
            if self.traffic_light_status == "red":
                self.publish_vel(0, 0)
                self.is_stopped = True

            elif self.traffic_light_status == "yellow":
                self.publish_vel(self.FULL_VELOCITY/2, self.line_angular_vel)
                self.sign_exist()

            else:
                self.publish_vel(self.FULL_VELOCITY, self.line_angular_vel)"""

    def drive(self, target, speed):

        if not self.driving:
            self.distance_time = rospy.get_time()
            self.driving = True

        self.publish_vel(speed, 0)
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

    def turn(self, angle, speed):
        """
            Turns the robot by a given angle.
        """
        if not self.turning:
            self.heading = 0
            self.turning = True

        self.update_heading()
        self.turn_error_pub.publish(angle - self.heading)
        self.publish_vel(0, self.turn_angular_vel*speed)

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

        print("[MAIN]  My battery is low and it's getting dark")

    def wl_cb(self, wl):
        self.wl = wl.data

    def wr_cb(self, wr):
        self.wr = wr.data

    def traffic_light_cb(self, msg):
        self.traffic_light_status = msg.name

    def line_angular_vel_cb(self, msg):
        self.line_angular_vel = msg.data

    def turn_angular_vel_cb(self, msg):
        self.turn_angular_vel = msg.data

    def crossroad_cb(self, msg):
        self.crossroad = msg.data

    def traffic_sign_cb(self, msg):
        self.traffic_sign = msg.name

    def yolo_started_cb(self, msg):
        self.yolo_started = msg.data


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    rospy.init_node("final_challenge", anonymous=True)
    Robot()
