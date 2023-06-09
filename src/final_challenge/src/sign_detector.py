#!/usr/bin/env python

# import ROS stuff
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from final_challenge.msg import sign_msg
# import the necessary packages
from collections import deque
import cv2
import rospy
import numpy as np

sign_array = []
classes = ['Forward', 'give_way', 'left', 'right', 'road_work', 'stop', 'traffic_green', 'traffic_red', 'traffic_yellow']

sign_message = sign_msg()

class SignDetector():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        ros_rate = rospy.Rate(50)

        self.sign_pub = rospy.Publisher(
            "/sign", String, queue_size=1)
        self.image_pub = rospy.Publisher(
            "/processed_image_sign", Image, queue_size=1)

        self.image_sub = rospy.Subscriber(
            "/camera/image_raw", Image, self.camera_callback)
        self.image_sub_jetson = rospy.Subscriber(
            "/video_source/raw", Image, self.camera_callback)
        self.image_sub_jetson_launch = rospy.Subscriber(
            "/camera/video_source/raw", Image, self.camera_callback)

        self.bridge_object = CvBridge()

        # This flag is to ensure we received at least one self.frame
        self.image_received_flag = 0

        while not rospy.is_shutdown():

            if self.image_received_flag == 1:
                # Detect sign and highlight it

                sign, processed_image = self.find_sign(self.frame)

                image_topic = self.bridge_object.cv2_to_imgmsg(
                    processed_image, encoding="bgr8")
                self.image_pub.publish(image_topic)

                if sign is not None:
                    self.sign_pub.publish(sign)

                self.image_received_flag = 0

            key = cv2.waitKey(1)
            ros_rate.sleep()

    def big_area(sign_array):
        if len(sign_array) > 1:
            print("More than one array received")
            area = []
            biggest_area = 0.0
            sign_area_array = []
            sign_array_send = []

            for i in range(len(sign_array)):
                data = sign_array[i, :-2]
                area = data[0] * data[1]
                sign_area_array.append(area)
            
            biggest_area = max(sign_area_array)
            index = sign_area_array.index(biggest_area)
            
            sign_array_send = sign_array[index]

            sign_message.sign_name = classes[0]
            sign_message.x = sign_array_send[1]
            sign_message.y = sign_array_send[2]
            sign_message.width = sign_array_send[3]
            sign_message.height = sign_array_send[4]



            return sign_message
        else:

            sign_message.sign_name = classes[0]
            sign_message.x = sign_array[1]
            sign_message.y = sign_array[2]
            sign_message.width = sign_array[3]
            sign_message.height = sign_array[4]

            return sign_message  

    def find_sign(self, frame):
        # Get sign
        sign = ""
        return sign, frame

    def camera_callback(self, data):
        try:
            self.frame = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
            self.image_received_flag = 1
        except CvBridgeError as e:
            print(e)

    def cleanup(self):
        print("Shutting down sign detector node")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('sign_detector', anonymous=True)
    print("Running sign detector")
    SignDetector()
