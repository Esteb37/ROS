#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from wang_mendel import wang_mendel


def stop():
    print("Stopping...")


def callback_robot_pose(r_pose):
    global x_pose, y_pose, z_orientation
    x_pose = r_pose.position.x
    y_pose = r_pose.position.y
    z_orientation = r_pose.orientation.z


def callback_goal(r_goal):
    global x_goal, y_goal
    x_goal = r_goal.position.x
    y_goal = r_goal.position.y


if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("controller")
    rospy.on_shutdown(stop)
    rate = rospy.Rate(50)

    # linear or angular
    mode = rospy.get_name().split("_")[0][1:]

    max_speed = rospy.get_param("/"+mode+"_controller_max_speed", 0)

    # Subscribers
    rospy.Subscriber("/robot_pose", Pose, callback_robot_pose)
    rospy.Subscriber("/goal", Pose, callback_goal)

    # Publishers
    pub = rospy.Publisher("/"+mode+"_vel", Float32, queue_size=10)

    distance_antecedents = rospy.get_param(
        "/"+mode+"_controller_distance_antecedents")
    angle_antecedents = rospy.get_param(
        "/"+mode+"_controller_angle_antecedents")
    output_antecedents = rospy.get_param(
        "/"+mode+"_controller_output_antecedents")

    rules = np.load(mode+"_rules.npy")

    controller = wang_mendel([distance_antecedents, angle_antecedents, output_antecedents],
                             (-5, 5),
                             (-np.pi, np.pi),
                             (-max_speed, max_speed),
                             rules)

    while not rospy.is_shutdown():

        dx = x_goal - x_pose
        dy = y_goal - y_pose

        distance = np.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dy, dx) - z_orientation
        angle = np.arctan2(np.sin(angle), np.cos(angle))

        result = controller.get_output(distance, angle)

        pub.publish(result)
        rate.sleep()
