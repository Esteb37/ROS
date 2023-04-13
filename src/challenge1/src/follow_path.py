#!/usr/bin/env python

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
import rospy


class FollowPath():

    MOVE_VELOCITY_MS = 0.4

    TURN_VELOCITY_RS = 0.4

    SQUARE = ((1.6, 0), (1.6, 1.2), (0, 1.2), (0, 0))

    heading = 0

    def __init__(self):

        rospy.on_shutdown(self.cleanup)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        rospy.Subscriber("distance_topic", Float32, self.message_cb)

        self.rate = rospy.Rate(20)

        print("Running...")

        is_finished = False

        while not rospy.is_shutdown():

            path = self.SQUARE

            self.follow_path(path)

    def follow_path(self, path):

        print("Following...\n")

        commands = self.generate_commands(path)

        print(commands)
        print()

        for command in commands:
            func, arg = command

            if func == "turn":
                self.turn_cmd(arg)

            elif func == "distance":
                self.move_cmd(arg)

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

            distance = np.sqrt((x)**2 + (x)**2)

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

        self.target_distance = msg.data

        self.init_time = rospy.get_time()

        print("I received this message in the callback: " +
              str(self.target_distance))

    def cleanup(self):
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)
        print("I'm dying, bye bye!!!")


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":

    rospy.init_node("simple_move", anonymous=True)
    FollowPath()
