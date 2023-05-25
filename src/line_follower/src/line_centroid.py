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

                # resize to 500x500
                frame_orig = imutils.resize(self.frame, width=500)

                # invert colors
                frame = cv2.bitwise_not(frame_orig)

                # convert to hsv
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # get black objects
                lower_black = (0, 0, 0)
                upper_black = (180, 255, 30)
                mask = cv2.inRange(frame, lower_black, upper_black)

                # find the line which can be a rectangle or a curved line
                line = cv2.findContours(
                    mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                line = imutils.grab_contours(line)

                # draw the line
                cv2.drawContours(frame_orig, line, -1, (0, 255, 0), 3)

                if len(line) == 0:
                    print("No line found")
                    continue

                M = cv2.moments(line[0])
                if M["m00"] != 0:

                    centroid = (int(M["m10"] / M["m00"]),int(M["m01"] / M["m00"]))
                else:
                    centroid = (0,0)

                # draw the centroid
                cv2.circle(frame_orig, (centroid), 5, (0, 0, 255), -1)

                # Get the centroid position relative to the center of the image
                centroid_x = centroid[0] - 250.0

                self.line_centroid_pub.publish(centroid_x)

                cv2.imshow("Frame", frame_orig)
                image_topic = self.bridge_object.cv2_to_imgmsg(
                    frame_orig, encoding="bgr8")
                self.image_pub.publish(image_topic)

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
