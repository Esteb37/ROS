#!/usr/bin/env python

# import ROS stuff
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import the necessary packages
from collections import deque
import cv2
import rospy
import numpy as np


class SignDetector():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        ros_rate = rospy.Rate(50)

        self.sign_pub = rospy.Publisher(
            "/sign", String, queue_size=1)
        self.image_pub = rospy.Publisher(
            "/processed_image_sign", Image, queue_size=1)

        self.image_sub = rospy.Subscriber(
            "/camera/image_raw", Image, self.camera_callback)
        self.image_sub_jetson = rospy.Subscriber(
            "/video_source/raw", Image, self.camera_callback)
        self.image_sub_jetson_launch = rospy.Subscriber(
            "/camera/video_source/raw", Image, self.camera_callback)

        self.bridge_object = CvBridge()

        # This flag is to ensure we received at least one self.frame
        self.image_received_flag = 0

        while not rospy.is_shutdown():

            if self.image_received_flag == 1:
                # Detect sign and highlight it

                sign, processed_image = self.find_sign(self.frame)

                image_topic = self.bridge_object.cv2_to_imgmsg(
                    processed_image, encoding="bgr8")
                self.image_pub.publish(image_topic)

                if sign is not None:
                    self.sign_pub.publish(sign)

                self.image_received_flag = 0

            key = cv2.waitKey(1)
            ros_rate.sleep()

    def find_sign(self, frame):
        # Get sign
        sign = ""
        return sign, frame

    def camera_callback(self, data):
        try:
            self.frame = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
            self.image_received_flag = 1
        except CvBridgeError as e:
            print(e)

    def cleanup(self):
        print("Shutting down sign detector node")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('sign_detector', anonymous=True)
    print("Running sign detector")
    SignDetector()
