#!/usr/bin/env python

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from challenge1.msg import script_select
import numpy as np
import rospy


class PathFollower():

    heading = 0

    def __init__(self):

        rospy.on_shutdown(self.cleanup)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        rospy.Subscriber("/script_select", script_select, self.message_cb)

        self.rate = rospy.Rate(50)

        print("Running...")

        self.is_finished = True  # Until message received

        while not rospy.is_shutdown():

            if not self.is_finished:
                self.is_finished = self.follow_path(self.PATH)

    def follow_path(self, path):

        commands = self.generate_commands(path)

        for command in commands:

            if rospy.is_shutdown():
                return False

            func, arg = command

            if func == "turn":
                self.turn_cmd(arg)

            elif func == "distance":
                self.move_cmd(arg)

        print("Finished!")

        return True

    def generate_commands(self, path):

        commands = []

        prev_x, prev_y = 0, 0

        heading = 0

        for coordinate in path:
            target_x, target_y = coordinate

            x, y = target_x - prev_x, target_y - prev_y

            angle = np.arctan2(y, x)

            turn = (angle - heading)

            if turn > np.pi:
                turn -= 2 * np.pi
            elif turn < -np.pi:
                turn += 2 * np.pi

            distance = np.sqrt((x)**2 + (y)**2)

            prev_x, prev_y = target_x, target_y

            heading += turn

            commands.append(("turn", turn))
            commands.append(("distance", distance))

        return commands

    def move_robot(self, velocity):
        vel = Twist()
        vel.linear.x = velocity
        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)
        self.rate.sleep()

    def turn_robot(self, velocity):
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = velocity
        self.cmd_vel_pub.publish(vel)
        self.rate.sleep()

    def move_cmd(self, distance):
        print("Moving " + str(distance) + " meters")

        direction = np.sign(distance)

        velocity = direction * self.MOVE_VELOCITY_MS

        init_time = rospy.get_time()

        current_distance = 0.0

        while abs(current_distance) < abs(distance):

            if rospy.is_shutdown():
                return

            current_distance = (
                rospy.get_time() - init_time) * velocity

            self.move_robot(velocity)

        self.move_robot(0.0)

    def turn_cmd(self, angle):

        print("Turning " + str(angle) + " radians")

        direction = np.sign(angle)

        velocity = direction * self.TURN_VELOCITY_RS

        init_time = rospy.get_time()

        current_angle = 0.0

        while abs(current_angle) < abs(angle):

            if rospy.is_shutdown():
                return

            current_angle = (
                rospy.get_time() - init_time) * velocity

            self.turn_robot(velocity)

        self.turn_robot(0.0)

    def message_cb(self, msg):

        script_select = msg.script_select
        type_select = msg.type_select
        value = msg.vel_or_time

        segments = 0

        if script_select == "SQUARE":
            length = msg.square_length

            self.PATH = ((length, 0), (length, length),
                         (0, length), (0, 0))

            segments = 4

            print(("Following a square of length " + str(length) + "m ") +
                  ("with a velocity of " + str(value) + " m/s" if type_select == "VELOCITY" else "in " + str(value) + " seconds"))

        elif script_select == "PATH":

            self.PATH = rospy.get_param("path")

            segments = len(self.PATH)

            print(("Following path ") +
                  ("with a velocity of " + str(value) + " m/s" if type_select == "VELOCITY" else "in " + str(value) + " seconds"))

        if type_select == "VELOCITY":
            self.MOVE_VELOCITY_MS = value
            self.TURN_VELOCITY_RS = value

        elif type_select == "TIME":
            self.MOVE_VELOCITY_MS = value/(segments*2)
            self.TURN_VELOCITY_RS = value/(segments*2)

        self.is_finished = False

    def cleanup(self):
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)
        print("My battery is low, and it's getting dark.")


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":

    rospy.init_node("path_follower", anonymous=True)
    PathFollower()
