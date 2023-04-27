#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
import cv2
import numpy as np

min_green = np.array([50, 220, 220])
max_green = np.array([60, 255, 255])
min_red = np.array([170, 220, 220])
max_red = np.array([180, 255, 255])
min_blue = np.array([110, 220, 220])
max_blue = np.array([120, 255, 255])


class Filter(object):

    def __init__(self):

        rospy.on_shutdown(self.cleanup)
        self.bridge_object = CvBridge()

        r = rospy.Rate(10)  # 10Hz
        self.image_sub = rospy.Subscriber(
            "camera/image_raw", Image, self.camera_callback)
        self.img_received = 0

        while not rospy.is_shutdown():
            if self.img_received:
                cv2.imshow('image', self.cv_image)

                image = cv2.resize(self.cv_image, (300, 300))
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask_r = cv2.inRange(hsv, min_red, max_red)

                res_r = cv2.bitwise_and(image, image, mask=mask_r)
                cv2.imshow('Original', image)
                cv2.imshow('Red', res_r)

            cv2.waitKey(1)
            r.sleep()

        cv2.destroyAllWindows()

    def cleanup(self):
        cv2.destroyAllWindows()

    def camera_callback(self, data):

        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")

        except CvBridgeError as e:
            print(e)

        self.img_received = 1


if __name__ == '__main__':

    rospy.init_node('load_image_2', anonymous=True)

    showing_image_object = Filter()
