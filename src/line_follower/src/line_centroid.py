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
            "/video_source/raw", Image, self.camera_callback)

        ros_rate = rospy.Rate(50)
        while not rospy.is_shutdown():

            if self.image_received_flag == 1:

                # TODO Get the centroid

                centroid = 0

                cv2.imshow("Frame", self.frame)
                image_topic = self.bridge_object.cv2_to_imgmsg(
                    self.frame, encoding="bgr8")
                self.image_pub.publish(image_topic)

                self.line_centroid_pub.publish(centroid)

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

        # resize the.self.frame, blur it, and convert it to the HSV color space
        self.frame = imutils.resize(self.frame, width=600)
        # blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
        median = cv2.medianBlur(self.frame, 7)
        hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

        # construct a mask for the color, then perform a series of dilations and
        # erosions to remove any small blobs left in the mask
        mask = cv2.inRange(hsv, colorLower, colorUpper)
        if secondLower is not None and secondUpper is not None:
            mask2 = cv2.inRange(hsv, secondLower, secondUpper)
            mask = cv2.bitwise_or(mask, mask2)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow("mask", mask)
        gray_blurred = cv2.blur(mask, (3, 3))

        detected_circles = cv2.HoughCircles(gray_blurred,
                                            cv2.HOUGH_GRADIENT, 1, 20, param1=50,
                                            param2=30, minRadius=self.min_radius, maxRadius=1000)

        if detected_circles is not None:
            print(len(detected_circles))
            # find the circle with the largest radius
            largest_circle = detected_circles[0][0]
            for circle in detected_circles[0]:
                if circle[2] > largest_circle[2]:
                    largest_circle = circle

            x, y, radius = largest_circle
            cv2.circle(self.frame, (x, y),
                       radius, (0, 255, 0), 2)
            cv2.circle(self.frame, (x, y),
                       2, (0, 0, 255), 3)
            return radius
        return 0

    def cleanup(self):
        print("Shutting down vision node")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('line_detector', anonymous=True)
    print("Running line detector")
    LineDetector()
