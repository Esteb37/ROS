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
            "/camera/image_raw", Image, self.camera_callback)

        ros_rate = rospy.Rate(50)
        while not rospy.is_shutdown():

            if self.image_received_flag == 1:

                resized = imutils.resize(self.frame, width=500)
                resized = resized[500/3*2:500, 0:500]

                hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, (0, 0, 0), (0,0,0))

                contours = cv2.findContours(
                    mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours = imutils.grab_contours(contours)

                centroid = (0, 0)
                if len(contours) > 0:
                    centers = []
                    for c in contours:
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                        else:
                            centroid = (0, 0)
                        centers.append(centroid)
                    # find the one that is closest to the center of the image horizontally
                    closest = centers[0]
                    for c in centers:
                        if abs(c[0] - 250) < abs(closest[0] - 250):
                            closest = c
                    centroid = closest

                else:
                    print("No line")


                cv2.drawContours(resized, contours, -1, (0, 255, 0), 3)
                cv2.circle(resized, centroid, 5, (0, 0, 255), -1)

                cv2.imshow("Frame", resized)

                image_topic = self.bridge_object.cv2_to_imgmsg(
                    self.frame, encoding="bgr8")
                self.image_pub.publish(image_topic)

                self.line_centroid_pub.publish(float(centroid[0]-250))

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
