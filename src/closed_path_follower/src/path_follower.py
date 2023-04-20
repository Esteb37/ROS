#!/usr/bin/env python

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from closed_path_follower.msg import script_select
import numpy as np
import rospy


class PathFollower():

    # Defines what percentage of the total time will be designated to moving, as opposed to turning. 0.7 means 70% of the time will be for moving and 30% for turning. A larger number means faster turns and slower movements, and viceversa.
    MOVE_TURN_RATIO = 0.7

    TRACK_WIDTH = 0.19
    WHEEL_RADIUS = 0.05

    MAX_MOVE_VELOCITY_MS = 0.72
    MAX_TURN_VELOCITY_RS = 7.5

    def __init__(self):

        self.move_velocity_ms = 0
        self.turn_velocity_rs = 0
        self.path = []

        self.current_x = 0
        self.current_y = 0
        self.current_heading = 0
        self.linear_vel = 0
        self.angular_vel = 0
        self.wl = 0
        self.wr = 0

        self.init_time = rospy.get_time()

        # Setup ROS node
        rospy.on_shutdown(self.cleanup)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/script_select", script_select, self.message_cb)
        rospy.Subscriber("/wl", Float32, self.wl_cb)
        rospy.Subscriber("/wr", Float32, self.wr_cb)
        self.rate = rospy.Rate(50)

        self.is_finished = True  # Until message received

        while rospy.get_time() == 0:
            print("No simulated time received")

        print("Running...")
        while not rospy.is_shutdown():

            # Run only once until the next message is received
            if not self.is_finished:
                self.is_finished = self.follow_path(self.path)

    def update_position(self):

        dt = rospy.get_time()-self.init_time

        self.linear_vel = self.WHEEL_RADIUS * (self.wr+self.wl)/2
        self.angular_vel = self.WHEEL_RADIUS * \
            (self.wr-self.wl)/self.TRACK_WIDTH

        theta = self.angular_vel * dt

        self.current_heading += np.arctan2(
            np.sin(theta), np.cos(theta))

        self.current_x += self.linear_vel * dt * np.cos(self.current_heading)
        self.current_y += self.linear_vel * dt * np.sin(self.current_heading)

        self.init_time = rospy.get_time()

    def follow_path(self, path):
        """
            Follows a path of coordinates, turning and moving as necessary.
        """

        # Obtain the array of commands based on the path
        commands = self.generate_commands(path)

        # For measuring elapsed time
        timer_start = rospy.get_time()

        # For each command, execute the corresponding function
        for command in commands:

            # Cancel if program is interrupted
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
        """ 
            Generates a list of commands to follow a path of coordinates.
            Each command is a tuple of the form (function, argument).
        """

        commands = []

        # Initialize robot at (0, 0) facing up
        prev_x, prev_y = 0, 0
        heading = 0

        # For each coordiante, obtain the angle to turn and the distance to move, in that order, and add them to the list of commands
        for coordinate in path:

            # Get coordinates relative to the previous coordinate
            target_x, target_y = coordinate
            x, y = target_x - prev_x, target_y - prev_y

            angle = np.arctan2(y, x)

            # Take into account where the robot is currently heading
            turn = (angle - heading)

            # Make the turn the shortest possible
            if turn > np.pi:
                turn -= 2 * np.pi
            elif turn < -np.pi:
                turn += 2 * np.pi

            # Get distance to move
            distance = np.sqrt((x)**2 + (y)**2)

            # Update current coordinates and heading
            prev_x, prev_y = target_x, target_y
            heading += turn

            commands.append(("turn", turn))
            commands.append(("distance", distance))

        return commands

    def publish_vel(self, linear, angular):
        """
            Sends a velocity command to the robot through the cmd_vel topic.
        """
        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular
        print(linear, angular)
        self.update_position()
        self.cmd_vel_pub.publish(vel)
        self.rate.sleep()

    def move_cmd(self, distance):
        """
            Moves the robot a certain distance.
        """

        print("Moving " + str(distance) + " meters")

        # Get velocity vector in the direction of the distance
        direction = np.sign(distance)
        velocity = direction * self.move_velocity_ms

        init_time = rospy.get_time()

        current_distance = 0.0

        # Move until the distance is reached
        while abs(current_distance) < abs(distance):

            # Cancel if program is interrupted
            if rospy.is_shutdown():
                return

            # Update current distance based on velocity and elapsed time
            current_distance = (
                rospy.get_time() - init_time) * velocity

            # Move the robot
            self.publish_vel(velocity, 0)

        # Stop the robot
        self.publish_vel(0, 0)

    def turn_cmd(self, angle):
        """
            Turns the robot a certain angle.
        """

        print("Turning " + str(angle) + " radians")

        # Get velocity vector in the direction of the angle
        direction = np.sign(angle)
        velocity = direction * self.turn_velocity_rs

        init_time = rospy.get_time()

        current_angle = 0.0

        # Turn until the angle is reached
        while abs(current_angle) < abs(angle):

            # Cancel if program is interrupted
            if rospy.is_shutdown():
                return

            # Update current angle based on velocity and elapsed time
            current_angle = (rospy.get_time() - init_time) * velocity

            # Turn the robot
            self.publish_vel(0, velocity)

        # Stop the robot
        self.publish_vel(0, 0)

    def message_cb(self, msg):
        """
            Callback function for the script_select topic.
            Receives a message with the selected script and the corresponding parameters, generates the path and obtains the velocity.
        """

        # SQUARE or PATH
        script_select = msg.script_select

        # VELOCITY or TIME
        type_select = msg.type_select

        if script_select == "SQUARE":

            length = msg.square_length

            # Generates a path in the form of a square of length length
            self.path = ((length, 0), (length, length),
                         (0, length), (0, 0))

            print("Following a square of length " + str(length) + "m")

        elif script_select == "PATH":

            self.path = rospy.get_param("path", [])

            print("Following path")

        # Get the total length and rotations that the robot needs to complete
        commands = self.generate_commands(self.path)
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

            # Get velocities directly
            self.move_velocity_ms = msg.move_velocity
            self.turn_velocity_rs = msg.turn_velocity

            # Estimate the total time based on the total distance and rotations, multiplied by each velocity
            total_time = (total_distance / self.move_velocity_ms) + (
                total_rotation / self.turn_velocity_rs)

            if self.MAX_MOVE_VELOCITY_MS < self.move_velocity_ms:
                self.move_velocity_ms = self.MAX_MOVE_VELOCITY_MS
                print("Move velocity exceeded maximum value. Setting to " +
                      str(self.move_velocity_ms) + "m/s")
            if self.MAX_TURN_VELOCITY_RS < self.turn_velocity_rs:
                self.turn_velocity_rs = self.MAX_TURN_VELOCITY_RS
                print("Turn velocity exceeded maximum value. Setting to " +
                      str(self.turn_velocity_rs) + "rad/s")

        elif type_select == "TIME":
            total_time = msg.total_time

            # Estimate velocity based on the total distance and rotations, divided by the total time designated to each action
            self.move_velocity_ms = total_distance / \
                (total_time * self.MOVE_TURN_RATIO)

            self.turn_velocity_rs = total_rotation / \
                (total_time * (1-self.MOVE_TURN_RATIO))

            if self.MAX_MOVE_VELOCITY_MS < self.move_velocity_ms:
                print("Move velocity has value of " + str(self.move_velocity_ms) + "m/s, which exceeds maximum value of " +
                      str(self.MAX_MOVE_VELOCITY_MS) + "m/s. Time is impossible to reach.")
                return

            if self.MAX_TURN_VELOCITY_RS < self.turn_velocity_rs:
                print("Turn velocity has value of " + str(self.turn_velocity_rs) + "rad/s, which exceeds maximum value of " +
                      str(self.MAX_TURN_VELOCITY_RS) + "rad/s. Time is impossible to reach.")
                return

        print("Move velocity: " + str(self.move_velocity_ms) + "m/s")
        print("Turn velocity: " + str(self.turn_velocity_rs) + "rad/s")
        print("Total time: " + str(total_time) + "s")

        # Activate flag to start following the path
        self.is_finished = False

    def cleanup(self):
        """
            Stops the robot when the program is interrupted.
        """
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)

        # See https://abc7chicago.com/mars-nasa-opportunity-rover/5137455/
        print("My battery is low, and it's getting dark.")

    def wl_cb(self, wl):
        self.wl = wl.data

    def wr_cb(self, wr):
        self.wr = wr.data


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":

    rospy.init_node("path_follower", anonymous=True)
    PathFollower()
