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
import imutils
import rospy
import numpy as np

class LightDetector():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.traffic_light_pub = rospy.Publisher(
            "traffic_light", String, queue_size=1)
        self.image_pub = rospy.Publisher(
            "/processed_image", Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            "/video_source/raw", Image, self.camera_callback)

        self.bridge_object = CvBridge()
        self.center_ros = Point()

        r_radius = 0
        g_radius = 0
        y_radius = 0
        # This flag is to ensure we received at least one self.frame
        self.image_received_flag = 0
        self.radius_ros = 0

        # define the lower and upper boundaries for each color
        # The parameters are also here, in case the program do not find the yaml file
        redColorLower = tuple(rospy.get_param(
            "/redColorLower", (90, 120, 170)))
        redColorUpper = tuple(rospy.get_param(
            "/redColorUpper", (180, 255, 255)))
        greenColorLower = tuple(rospy.get_param(
            "/greenColorLower", (40, 20, 0)))
        greenColorUpper = tuple(rospy.get_param(
            "/greenColorUpper", (80, 255, 255)))
        yellowColorLower = tuple(rospy.get_param(
            "/yellowColorLower", (20, 120, 190)))
        yellowColorUpper = tuple(rospy.get_param(
            "/yellowColorUpper", (60, 255, 255)))
        secondRedColorLower = tuple(rospy.get_param(
            "/secondRedColorLower", (20, 120, 190)))
        secondRedColorUpper = tuple(rospy.get_param(
            "/secondRedColorUpper", (20, 120, 190)))

        self.min_radius = rospy.get_param("/min_radius", 20)
        self.pts = deque(maxlen=64)

        ros_rate = rospy.Rate(50)
        while not rospy.is_shutdown():

            if self.image_received_flag == 1:
                r_radius = self.find_ball(
                    redColorLower, redColorUpper, secondRedColorLower, secondRedColorUpper)
                g_radius = self.find_ball(greenColorLower, greenColorUpper)
                y_radius = self.find_ball(yellowColorLower, yellowColorUpper)
                print("Red r: " + str(r_radius) + " Green r: " +
                      str(g_radius) + " Yellow r: " + str(y_radius))

                cv2.imshow("Frame", self.frame)

                image_topic = self.bridge_object.cv2_to_imgmsg(
                    self.frame, encoding="bgr8")
                self.image_pub.publish(image_topic)

                self.image_received_flag = 0

                # Publish the color of the traffic light
                if r_radius or g_radius or y_radius:
                    if r_radius > g_radius and r_radius > y_radius:
                        self.traffic_light_pub.publish("red")
                    elif g_radius > r_radius and g_radius > y_radius:
                        self.traffic_light_pub.publish("green")
                    else:
                        self.traffic_light_pub.publish("yellow")
                else:
                    self.traffic_light_pub.publish("none")

            key = cv2.waitKey(1) & 0xFF
            ros_rate.sleep()

    def camera_callback(self, data):
        try:
            self.frame = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
            self.image_received_flag = 1
        except CvBridgeError as e:
            print(e)

    def find_ball(self, colorLower, colorUpper, secondLower=None, secondUpper=None):
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

        cv2.imshow("Mask",mask)
        # Blur using 3 * 3 kernel.
        gray_blurred = cv2.blur(mask, (3, 3))

        # Apply Hough transform on the blurred image.
        detected_circles = cv2.HoughCircles(gray_blurred,
                        cv2.HOUGH_GRADIENT, 1, 20, param1 = 50,
                    param2 = 30, minRadius = self.min_radius, maxRadius = 1000)

        if detected_circles is not None:
            circle = detected_circles[0][0]
            x, y, radius = circle
            cv2.circle(self.frame, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
            cv2.circle(self.frame, (int(x),int(y)), 5, (0, 0, 255), -1)
        else:
            return 0
        return radius


    def cleanup(self):
        print("Shutting down vision node")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('light_detector', anonymous=True)
    print("Running light detector")
    LightDetector()
