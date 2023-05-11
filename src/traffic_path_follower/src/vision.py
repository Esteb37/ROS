#!/usr/bin/env python

# import ROS stuff
import rospy
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import rospy


class BallTracker():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        
        rospy.init_node("vision")
        self.traffic_light_pub = rospy.Publisher("traffic_light", String)
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.camera_callback)

        self.bridge_object = CvBridge()
        self.center_ros = Point()

        r_radius = 0
        g_radius = 0
        y_radius = 0
        self.image_received_flag = 0 #This flag is to ensure we received at least one self.frame
        self.radius_ros = 0

        ap = argparse.ArgumentParser()
        ap.add_argument("-v", "--video",
            help="path to the (optional) video file")
        ap.add_argument("-b", "--buffer", type=int, default=64,
            help="max buffer size")
        self.args = vars(ap.parse_args())


        # define the lower and upper boundaries for each color
        #The parameters are also here, in case the program do not find the yaml file
        redColorLower = rospy.get_param("/redColorLower", (90, 120, 170))
        redColorUpper = rospy.get_param("/redColorUpper", (180, 255, 255))
        greenColorLower = rospy.get_param("/greenColorLower", (40, 20, 0))
        greenColorUpper = rospy.get_param("/greenColorUpper", (80, 255, 255))
        yellowColorLower = rospy.get_param("/yellowColorLower", (20, 120, 190))
        yellowColorUpper = rospy.get_param("/yellowColorUpper", (60, 255, 255))

        self.min_radius = rospy.get_param("/min_radius", 20)
        self.pts = deque(maxlen=self.args["buffer"])

        
        ros_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            if self.image_received_flag == 1:
                r_radius = self.find_ball(redColorLower, redColorUpper)
                ros_rate.sleep()
                g_radius = self.find_ball(greenColorLower, greenColorUpper)
                ros_rate.sleep()
                y_radius = self.find_ball(yellowColorLower, yellowColorUpper)
                print("Red r: " + str(r_radius) + " Green r: " + str(g_radius) + " Yellow r: " + str(y_radius))

                cv2.imshow("Frame", self.frame)
                self.image_received_flag = 0

                #Publish the color of the traffic light
                if r_radius or g_radius or y_radius:
                    if r_radius > g_radius and r_radius > y_radius: 
                        self.traffic_light_pub.publish("Red")
                    elif g_radius > r_radius and g_radius > y_radius:
                        self.traffic_light_pub.publish("Green")
                    else:
                        self.traffic_light_pub.publish("Yellow")
                else:
                    self.traffic_light_pub.publish(None)
            
            key = cv2.waitKey(1) & 0xFF
            ros_rate.sleep()


    def camera_callback(self, data):
        try:
            self.frame = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.image_received_flag = 1
        except CvBridgeError as e:
            print(e)


    def find_ball(self, colorLower, colorUpper):
        # resize the.self.frame, blur it, and convert it to the HSV color space
        self.frame = imutils.resize(self.frame, width=600)
        #blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
        median = cv2.medianBlur(self.frame,7)
        hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

        # construct a mask for the color, then perform a series of dilations and
        # erosions to remove any small blobs left in the mask
        mask = cv2.inRange(hsv, colorLower, colorUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use it to 
            # compute the minimum enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > self.min_radius:
                # draw the circle and centroid on the self.frame,
                # then update the list of tracked points
                self.center_ros.x=float(x)
                self.center_ros.y=float(y)
                self.center_ros.z=0 #As it is an self.frame z is not used.
                self.radius_ros=int(radius)

                cv2.circle(self.frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(self.frame, center, 5, (0, 0, 255), -1)
            else:
                self.center_ros.x=0
                self.center_ros.y=0
                self.center_ros.z=0
                self.radius_ros=0
                cv2.circle(self.frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(self.frame, center, 5, (0, 0, 255), -1)
        else:
            # Publish a radius of zero if there is no detected object
            self.center_ros.x=0
            self.center_ros.y=0
            self.center_ros.z=0
            self.radius_ros=0
            cv2.circle(self.frame, (0, 0), 1, (0, 0, 0), 2)

        # update the points queue
        self.pts.appendleft(center)

        # loop over the set of tracked points
        for i in range(1, len(self.pts)):
            # if either of the tracked points are None, ignore them
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue
        
        return self.radius_ros

    
    def cleanup(self):
        print("Shutting down vision node")
        cv2.destroyAllWindows()
        

if __name__ == '__main__':
    #rospy.init_node('vision', anonymous=True)
    BallTracker()
