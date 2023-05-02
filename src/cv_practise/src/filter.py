#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
import cv2
import numpy as np

H_min = 0  # HUE (0-180) = (0-360)
S_min = 0  # SATURATION (0-255)
V_min = 0  # VALUE (0-255)

H_max = 180
S_max = 255
V_max = 255


def save_value(name, value):
    with open("/home/estebanp/catkin_ws/src/cv_practise/src/hsv_values.txt", "r+") as f:
        # Find the line and replace it with the new value
        lines = f.readlines()
        for i, line in enumerate(lines):
            if name in line:
                lines[i] = line.replace(line.split(":")[1], str(value)+"\n")

        f.seek(0)
        f.truncate()
        f.writelines(lines)


def get_value(name):
    try:
        with open("/home/estebanp/catkin_ws/src/cv_practise/src/hsv_values.txt", "r") as f:
            # Find the line with the name and read the value
            lines = f.readlines()
            for line in lines:
                if name in line:
                    return int(line.split(":")[1])
    except:
        return 0


cv2.namedWindow("Sliders")
cv2.resizeWindow("Sliders", 500, 200)

cv2.createTrackbar("H Min", "Sliders", 0, 180,
                   lambda x: save_value("H_min", x))
cv2.setTrackbarPos("H Min", "Sliders", get_value("H_min"))
cv2.createTrackbar("H Max", "Sliders", 0, 180,
                   lambda x: save_value("H_max", x))
cv2.setTrackbarPos("H Max", "Sliders", get_value("H_max"))

cv2.createTrackbar("S Min", "Sliders", 0, 255,
                   lambda x: save_value("S_min", x))
cv2.setTrackbarPos("S Min", "Sliders", get_value("S_min"))
cv2.createTrackbar("S Max", "Sliders", 0, 255,
                   lambda x: save_value("S_max", x))
cv2.setTrackbarPos("S Max", "Sliders", get_value("S_max"))

cv2.createTrackbar("V Min", "Sliders", 0, 255,
                   lambda x: save_value("V_min", x))
cv2.setTrackbarPos("V Min", "Sliders", get_value("V_min"))
cv2.createTrackbar("V Max", "Sliders", 0, 255,
                   lambda x: save_value("V_max", x))
cv2.setTrackbarPos("V Max", "Sliders", get_value("V_max"))


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

                H_min = cv2.getTrackbarPos("H Min", "Sliders")
                H_max = cv2.getTrackbarPos("H Max", "Sliders")
                S_min = cv2.getTrackbarPos("S Min", "Sliders")
                S_max = cv2.getTrackbarPos("S Max", "Sliders")
                V_min = cv2.getTrackbarPos("V Min", "Sliders")
                V_max = cv2.getTrackbarPos("V Max", "Sliders")

                image = cv2.resize(self.cv_image, (300, 300))
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask_r = cv2.inRange(
                    hsv, (H_min, S_min, V_min), (H_max, S_max, V_max))

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
