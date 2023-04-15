#!/usr/bin/env python

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from challenge1.msg import script_select
import numpy as np
import rospy


class PathFollower():

    MOVE_TURN_RATIO = 0.7

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

        timer_start = rospy.get_time()

        for command in commands:

            if rospy.is_shutdown():
                return False

            func, arg = command

            if func == "turn":
                self.turn_cmd(arg)

            elif func == "distance":
                self.move_cmd(arg)

        print("Finished in " + str(rospy.get_time() - timer_start) + " seconds")

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

    def publish_vel(self, linear, angular):
        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular
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

            self.publish_vel(velocity, 0)

        self.publish_vel(0, 0)

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

            self.publish_vel(0, velocity)

        self.publish_vel(0, velocity)

    def message_cb(self, msg):

        script_select = msg.script_select
        type_select = msg.type_select

        if script_select == "SQUARE":

            length = msg.square_length

            self.PATH = ((length, 0), (length, length),
                         (0, length), (0, 0))

            print("Following a square of length " + str(length) + "m")

        elif script_select == "PATH":

            self.PATH = rospy.get_param("path", [])

            print("Following path")

        commands = self.generate_commands(self.PATH)

        total_time = 0
        total_rotation = 0
        total_distance = 0
        for command in commands:
            func, arg = command

            if func == "turn":
                total_rotation += abs(arg)
            elif func == "distance":
                total_distance += abs(arg)

        if type_select == "VELOCITY":
            self.MOVE_VELOCITY_MS = msg.move_velocity
            self.TURN_VELOCITY_RS = msg.turn_velocity

            total_time = (total_distance / self.MOVE_VELOCITY_MS) + (
                total_rotation / self.TURN_VELOCITY_RS)

        elif type_select == "TIME":
            total_time = msg.total_time

            self.MOVE_VELOCITY_MS = total_distance / \
                (total_time * self.MOVE_TURN_RATIO)

            self.TURN_VELOCITY_RS = total_rotation / \
                (total_time * (1-self.MOVE_TURN_RATIO))

        print("Move velocity: " + str(self.MOVE_VELOCITY_MS) + "m/s")
        print("Turn velocity: " + str(self.TURN_VELOCITY_RS) + "rad/s")
        print("Total time: " + str(total_time) + "s")

        self.is_finished = False

    def cleanup(self):
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)
        print("My battery is low, and it's getting dark.")


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":

    rospy.init_node("path_follower", anonymous=True)
    PathFollower()
