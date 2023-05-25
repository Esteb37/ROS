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
                """
                    Hay que encontrar la línea con un filtro o algo
                    Una vez encontrada la línea, se calcula la posición del centroide
                    Solo nos interesa la posición del centroide en el eje horizontal,
                    porque queremos centrar el robot en la línea
                    Estaría bien que implementáramos algo para que si detecta una segunda
                    línea, la ignore para que se mantenga sobre la correcta
                """

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

    def cleanup(self):
        print("Shutting down vision node")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('line_detector', anonymous=True)
    print("Running line detector")
    LineDetector()
