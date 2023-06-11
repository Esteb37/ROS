#!/usr/bin/env python

# import ROS stuff
import rospy
from std_msgs.msg import Float32MultiArray
from final_challenge.msg import detected_object
import numpy as np

class SignDetector():

    MIN_DETECTION_AREA = 1000

    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        self.sign_pub = rospy.Publisher(
            "/sign", detected_object, queue_size=1)

        self.yolo_sub = rospy.Subscriber("/yolo_results", Float32MultiArray, self.yolo_cb)

        self.yolo_matrix = []
        ros_rate = rospy.Rate(50)

        categories = ["forward", "give_way", "left", "right", "road_work", "stop", "green", "red", "yellow", "none"]

        while not rospy.is_shutdown():

            closest_sign = "none"
            min_area = 0

            if len(self.yolo_matrix):

                for sign in self.yolo_matrix:

                    width = sign[2] - sign[0]
                    height = sign[3] - sign[1]

                    category = categories[int(sign[5])]

                    area = width * height
                    print(category, area)

                    if area > self.MIN_DETECTION_AREA and area > min_area:
                        min_area = area
                        closest_sign = category

            self.sign_pub.publish(closest_sign, min_area)

            ros_rate.sleep()

    def yolo_cb(self, data):
        flat = data.data
        self.yolo_matrix = np.reshape(flat, (-1, 6))

    def cleanup(self):
        print("Shutting down sign node")


if __name__ == '__main__':
    rospy.init_node('sign_detector', anonymous=True)
    print("Running sign detector")
    SignDetector()
