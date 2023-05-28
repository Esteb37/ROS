#!/usr/bin/env python

# import ROS stuff
import rospy
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import the necessary packages
from collections import deque
import cv2

import rospy
import numpy as np


def condense_consecutive_ones(arr):
    found_one = False
    for i in range(len(arr)):
        if arr[i] == 1:
            if found_one:
                arr[i] = 0
            else:
                found_one = True
        else:
            found_one = False
    return arr


class LineDetector():
    def __init__(self):

        self.bridge_object = CvBridge()

        self.image_received_flag = 0

        self.line_threshold = 100

        rospy.on_shutdown(self.cleanup)
        self.line_centroid_pub = rospy.Publisher(
            "/line_centroid", Float32, queue_size=1)
        self.image_pub = rospy.Publisher(
            "/processed_image_line", Image, queue_size=1)

        self.image_sub = rospy.Subscriber(
            "/camera/image_raw", Image, self.camera_callback)
        self.image_sub_jetson = rospy.Subscriber(
            "/video_source/raw", Image, self.camera_callback)
        self.image_sub_jetson_launch = rospy.Subscriber(
            "/camera/video_source/raw", Image, self.camera_callback)

        self.threshold_sub = rospy.Subscriber(
            "/line_threshold", Int32, self.threshold_callback)

        ros_rate = rospy.Rate(50)

        last_centroid = (250, 250)

        while not rospy.is_shutdown():

            if self.image_received_flag == 1:

                resized = cv2.resize(self.frame, (500, 500))[
                    500/3*2:500, 0:500]
                gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

                # gaussian blurr
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                # dilate and erode
                blurred = cv2.erode(cv2.dilate(
                    blurred, None, iterations=2), None, iterations=2)

                _, thresh = cv2.threshold(
                    blurred, self.line_threshold, 255, cv2.THRESH_BINARY_INV)

                cnts, _ = cv2.findContours(thresh.copy(), cv2.RETR_CCOMP,
                                           cv2.CHAIN_APPROX_SIMPLE)
                # filter out small contours
                cnts = [c for c in cnts if cv2.contourArea(c) > 100]

                centers = []
                for c in cnts:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        centers.append(
                            (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])))
                    else:
                        centers.append((0, 0))
                for center in centers:
                    cv2.circle(thresh, center, 5, (0, 0, 255), -1)

                # turn thresh into color image
                thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

                cv2.drawContours(thresh, cnts, -1, (0, 255, 0), 2)

                min_dist = 100000
                for center in centers:
                    dist = np.linalg.norm(
                        np.array(last_centroid) - np.array(center))
                    if dist < min_dist:
                        min_dist = dist
                        min_center = center
                last_centroid = min_center

                cv2.circle(thresh, min_center, 5, (255, 0, 0), -1)

                # cv2.imshow("output", thresh)

                image_topic = self.bridge_object.cv2_to_imgmsg(
                    thresh, encoding="bgr8")
                self.image_pub.publish(image_topic)

                self.line_centroid_pub.publish(min_center[0] - 250)

                self.image_received_flag = 0

            key = cv2.waitKey(1) & 0xFF
            ros_rate.sleep()

    def camera_callback(self, data):
        try:
            self.frame = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
            self.image_received_flag = 1
        except CvBridgeError as e:
            print(e)

    def threshold_callback(self, data):
        self.line_threshold = data.data

    def cleanup(self):
        print("Shutting down vision node")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('line_detector', anonymous=True)
    print("Running line detector")
    LineDetector()
