#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
import sys

e = [0, 0, 0]
u = [0, 0]

error = 0

def stop():
    print("Stopping...")


def callback_error(err):
    global error
    error = err.data


if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("controller")
    rospy.on_shutdown(stop)
    rate = rospy.Rate(50)

    if len(sys.argv) < 3:
        print("Usage: angular_controller.py <error_topic> <vel_topic>")
        sys.exit(1)


    error_topic = sys.argv[1]
    vel_topic = sys.argv[2]

    kP = rospy.get_param("/"+error_topic+"_controller_kP", 0)
    kI = rospy.get_param("/"+error_topic+"_controller_kI", 0)
    kD = rospy.get_param("/"+error_topic+"_controller_kD", 0)
    Ts = rospy.get_param("/controller_Ts", 0.02)

    K1 = kP + Ts * kI + kD / Ts
    K2 = -kP - 2.0 * kD / Ts
    K3 = kD / Ts

    max_speed = rospy.get_param("/"+error_topic+"_controller_max_speed", 0)

    # Subscribers
    rospy.Subscriber("/"+error_topic, Float32, callback_error)

    # Publishers
    pub = rospy.Publisher("/"+vel_topic, Float32, queue_size=10)



    while not rospy.is_shutdown():


        e[0] = error

        u[0] = K1 * e[0] + K2 * e[1] + K3 * e[2] + u[1]

        e[2] = e[1]
        e[1] = e[0]
        u[1] = u[0]

        result = np.clip(u[0], -max_speed, max_speed)
        pub.publish(result)
        rate.sleep()
