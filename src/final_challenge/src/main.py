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
                    ("turn", -np.pi/2, 0.8),
                    ("drive",0.3, FULL_VELOCITY*0.8)]

    TURN_LEFT_CMD = [("drive",0.2, FULL_VELOCITY*0.8),
                     ("turn", np.pi/2, 0.8),
                     ("drive",0.3, FULL_VELOCITY*0.8)]

    DRIVE_FORWARD_CMD = [("drive", 0.5, FULL_VELOCITY * 0.8)]


    CROSSROAD_OBEY_THRESHOLD = 150

    SIGN_OBEY_THRESHOLD = 800

    LIGHT_OBEY_THRESHOLD = 500

    STOP_TIME = 10

    GIVE_WAY_TIME = 5

    ROAD_WORK_TIME = 2

    LOGGING = True

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
        self.traffic_sign = detected_object("none", 0)
        self.action_sign = detected_object("none", 0)
        self.traffic_light = detected_object("none", 0)
        self.last_sign = detected_object("none", 0)

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

        self.LOG("Running...")

        self.current_action = 0

        while not rospy.is_shutdown():

            if self.crossing:
                self.LOG("Crossing")
                self.cross_road()
            else:
                self.crossing = self.crossroad > self.CROSSROAD_OBEY_THRESHOLD
                (linear_vel, angular_vel) = self.obey_traffic()
                self.publish_vel(linear_vel, angular_vel)

            self.crossing_pub.publish(self.crossing)
            self.rate.sleep()

    def LOG(self, msg):
        if self.LOGGING:
            print("[MAIN] " + msg)

    def setup_node(self):
        rospy.on_shutdown(self.cleanup)
        self.rate = rospy.Rate(50)
        self.setup_publishers()
        self.setup_subscribers()

        # Setup ROS node
        self.LOG("Waiting for time to be set...")
        while rospy.get_time() == 0:
            pass

        self.LOG("Waiting for YOLO...")
        while not self.yolo_started:
            pass

    def setup_publishers(self):
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size = 10)
        self.turn_error_pub = rospy.Publisher(
            '/turn_error', Float32, queue_size = 10)
        self.crossing_pub = rospy.Publisher(
            "/crossing", Bool, queue_size = 10)

    def setup_subscribers(self):
        rospy.Subscriber("/wl", Float32, self.wl_cb)
        rospy.Subscriber("/wr", Float32, self.wr_cb)
        rospy.Subscriber("/traffic_light", detected_object, self.traffic_light_cb)
        rospy.Subscriber("/line_angular_vel", Float32,
                         self.line_angular_vel_cb)
        rospy.Subscriber("/turn_angular_vel", Float32,
                         self.turn_angular_vel_cb)
        rospy.Subscriber("/crossroad", Int32, self.crossroad_cb)
        rospy.Subscriber("/sign", detected_object, self.traffic_sign_cb)
        rospy.Subscriber("/yolo_started", Bool, self.yolo_started_cb)


    def cross_road(self):

        if self.action_sign.name == "left":
            command = self.TURN_LEFT_CMD
        elif self.action_sign.name == "right":
            command = self.TURN_RIGHT_CMD
        else:
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
            self.LOG("Done crossing")

    def sign_is(self, sign, min_area = SIGN_OBEY_THRESHOLD):
        is_sign = (self.traffic_sign.name == sign
                and self.traffic_sign.area > min_area
                and self.traffic_sign.name != self.last_sign.name)
        if is_sign:
            self.last_sign = self.traffic_sign
        return is_sign

    def light_is(self, light, min_area = LIGHT_OBEY_THRESHOLD):
        return self.traffic_light.name == light and self.traffic_light.area > min_area

    def obey_traffic(self):
        if self.light_is("red"):
            self.LOG("Stopped at red")
            return (0, 0)

        if self.light_is("yellow"):
            self.LOG("Slowing at yellow.")
            return (self.FULL_VELOCITY / 2, self.line_angular_vel)

        if self.sign_is("stop") and not self.stopping and self.stop_released:
            self.LOG("Stopped at sign.")
            self.stopping_time = rospy.get_time()
            self.stopping = True

        if self.stopping:
            if rospy.get_time() - self.stopping_time > self.STOP_TIME:
                self.stopping = False
                self.stop_released = False
                self.LOG("Stopped finished.")
            return (0, 0)

        if not self.sign_is("stop"):
            self.stop_released = True

        if self.sign_is("give_way") and not self.giving_way:
            self.LOG("Giving way.")
            self.giving_way_time = rospy.get_time()
            self.giving_way = True

        if self.giving_way:
            if rospy.get_time() - self.giving_way_time > self.GIVE_WAY_TIME:
                self.giving_way = False
                self.LOG("Giving way finished.")
            return (self.FULL_VELOCITY / 2, self.line_angular_vel)

        if self.sign_is("road_work") and not self.road_work:
            self.LOG("Slowing for work.")
            self.road_work_time = rospy.get_time()
            self.road_work = True

        if self.road_work:
            if rospy.get_time() - self.road_work_time > self.road_work:
                self.road_work = False
                self.LOG("Road work finished.")
            return (self.FULL_VELOCITY / 2, self.line_angular_vel)

        if (self.sign_is("right", min_area = 500)
            or self.sign_is("left", min_area = 500)
            or self.sign_is("forward", min_area = 500)):

            self.action_sign = self.traffic_sign

        return (self.FULL_VELOCITY, self.line_angular_vel)

    def drive(self, target, speed):

        if not self.driving:
            self.LOG("Driving forward for {}m".format(target))
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
            self.LOG("Turning {} rads".format(angle))
            self.heading = 0
            self.turning = True

        self.update_heading()

        error = angle - self.heading
        error = np.arctan2(np.sin(error), np.cos(error))

        self.turn_error_pub.publish(error)
        self.publish_vel(0, self.turn_angular_vel*speed)

        if np.abs(error) < 0.1:
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

        self.LOG("My battery is low and it's getting dark")

    def wl_cb(self, wl):
        self.wl = wl.data

    def wr_cb(self, wr):
        self.wr = wr.data

    def line_angular_vel_cb(self, msg):
        self.line_angular_vel = msg.data

    def turn_angular_vel_cb(self, msg):
        self.turn_angular_vel = msg.data

    def crossroad_cb(self, msg):
        self.crossroad = msg.data

    def traffic_sign_cb(self, msg):
        self.traffic_sign = msg

    def traffic_light_cb(self, msg):
        self.traffic_light = msg


    def yolo_started_cb(self, msg):
        self.yolo_started = msg.data


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    rospy.init_node("final_challenge", anonymous=True)
    Robot()
