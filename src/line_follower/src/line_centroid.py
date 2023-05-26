#!/usr/bin/env python

# import ROS stuff
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import the necessary packages
from collections import deque
import cv2
import imutils
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

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

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)


class LineDetector():
    def __init__(self):

        self.bridge_object = CvBridge()

        self.image_received_flag = 0

        rospy.on_shutdown(self.cleanup)
        self.line_centroid_pub = rospy.Publisher(
            "/line_centroid", Float32, queue_size=1)
        self.image_pub = rospy.Publisher(
            "/processed_image", Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw", Image, self.camera_callback)

        ros_rate = rospy.Rate(50)


        while not rospy.is_shutdown():

            if self.image_received_flag == 1:

                resized = imutils.resize(self.frame, width=500)[500/3*2:500, 0:500]
                gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

                # gaussian blur
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                # dilate and erode
                thresh = cv2.erode(cv2.dilate(blurred, None, iterations=2), None, iterations=2)

                # average light on the vertical axis
                avg = np.average(thresh, axis=0)

                gradient = np.gradient(avg)

                second_derivative = np.gradient(gradient)

                pos_gradient = np.where(gradient > 10, gradient, 0)
                neg_gradient = np.where(gradient < -10, -gradient, 0)

                pos_product = pos_gradient * second_derivative
                neg_product = neg_gradient * second_derivative

                pos_product = np.where(pos_product > 0, pos_product, 0)
                neg_product = np.where(neg_product > 0, neg_product, 0)

                pos_shifted = np.roll(pos_product, -1)
                pos_edges = np.where(pos_shifted > pos_gradient, 1, 0)

                neg_shifted = np.roll(neg_product, -1)
                neg_edges = np.where(neg_shifted > neg_gradient, 1, 0)

                pos_edges = condense_consecutive_ones(pos_edges)
                neg_edges = condense_consecutive_ones(neg_edges)


                cv2.imshow("output", resized)

                plt.plot(pos_edges, 'ro')
                plt.plot(neg_edges, 'bo')
                plt.show()


                image_topic = self.bridge_object.cv2_to_imgmsg(
                    self.frame, encoding="bgr8")
                self.image_pub.publish(image_topic)

                self.line_centroid_pub.publish(0)

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

    def cleanup(self):
        print("Shutting down vision node")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('line_detector', anonymous=True)
    print("Running line detector")
    LineDetector()
