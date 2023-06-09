#!/usr/bin/env python

# import ROS stuff
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from final_challenge.msg import sign_msg
# import the necessary packages
from collections import deque
import cv2
import rospy
import numpy as np


class SignalGenerator():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        ros_rate = rospy.Rate(50)  # 50 Hz

        self.sign_generator_pub = rospy.Publisher("/sign", String, queue_size=1)


        while not rospy.is_shutdown():
            sign = input("Sign: ")

            if sign is not None:
                self.publish_sign(sign)

            ros_rate.sleep()

    def publish_sign(self, sign):
        # Publish the sign message
        self.sign_generator_pub.publish(sign)
        print(sign)

    def cleanup(self):
        print("Shutting down vision node")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('signal_generator', anonymous=True)  # Initialize the ROS node
    print("Running signal generator node.")
    SignalGenerator()

