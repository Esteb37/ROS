#!/usr/bin/env python3
import cv2
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
import rospy
import time
import sys

print("Importing yolo... ")

HOME = "/"+sys.path[0].split("/")[1]+"/"+sys.path[0].split("/")[2]
PACKAGE_PATH = sys.path[0][:-3]

sys.path.append(HOME+'/yolov5')
from detection import detector
print("Imported yolo!")

def string_to_rgb_color(string):
    # Compute the hash value of the string
    hash_value = hash(string)

    # Extract the RGB components from the hash value
    red = (hash_value & 0xFF0000) >> 16
    green = (hash_value & 0x00FF00) >> 8
    blue = hash_value & 0x0000FF

    return (blue, green, red)

def image_to_msg(image):
    # Convert the image to a ROS Image message

    if len(image.shape) == 3:
        height, width, channels = image.shape
        encoding = 'bgr8'  # Specify the image encoding
    else:
        height, width = image.shape
        encoding = "mono8"
        channels = 1

    # Create the Image message
    img_msg = Image()
    img_msg.header.stamp = rospy.Time.now()
    img_msg.header.frame_id = 'camera_frame'
    img_msg.height = height
    img_msg.width = width
    img_msg.encoding = encoding
    img_msg.step = width * channels
    img_msg.data = image.tobytes()

    return img_msg

def msg_to_image(msg):
    width = msg.width
    height = msg.height
    channels = msg.step // msg.width
    img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, channels))
    return img_data


class Robot():

    frame = None
    pred = []

    def __init__(self):
        self.WIDTH = 224

        self.setup_node()
        self.image_received_flag = 0

        # folder path

        weights = PACKAGE_PATH+"/models/best.onnx"

        print("Loading network...")
        t = time.time()
        yolo = detector(weights, 0.5)
        print("Loaded in", time.time() - t)

        self.names = ["forward", "give_way", "left", "right", "road_work", "stop", "green", "red", "yellow"]



        time_avg = 0
        time_count = 0

        print_time = time.time()
        data_sender = Float32MultiArray()

        print("Reading... ")
        while not rospy.is_shutdown():
            if self.frame is not None and self.image_received_flag == 1:
                t = time.time()
                #try:

                self.pred = yolo.detect(self.frame)
                time_avg += time.time()-t
                time_count += 1

                if time.time() - print_time > 5:
                    print("Latency: ", time_avg/time_count)
                    time_avg = 0
                    time_count = 0
                    print_time = time.time()

                empty = True
                for det in self.pred:
                    for d in det:
                        empty = False
                        data_sender.data = d
                        self.yolo_pub.publish(data_sender)

                if empty:
                    data_sender.data = []
                    self.yolo_pub.publish(data_sender)

                #except Exception as e:
                    #print(e)

            self.image_received_flag == 0

    def setup_node(self):
        rospy.on_shutdown(self.cleanup)
        self.rate = rospy.Rate(50)

        self.image_sub = rospy.Subscriber(
        "/video_source/raw", Image, self.camera_callback)
        self.image_sub = rospy.Subscriber(
        "/camera/video_source/raw", Image, self.camera_callback)

        self.image_pub = rospy.Publisher("/processed_image_yolo", Image, queue_size=1)
        self.yolo_pub = rospy.Publisher("/yolo_results", Float32MultiArray, queue_size = 100)

    def camera_callback(self, data):
        #try:
        if self.frame is not None:
            for _, det in enumerate(self.pred):
                for d in det:
                    top_left = (int(d[0]), int(d[1]))
                    bottom_right = (int(d[2]), int(d[3]))
                    class_name = self.names[int(d[5])]

                    color_hash = string_to_rgb_color(class_name)

                    cv2.rectangle(self.frame, top_left, bottom_right, color_hash, 2)
                    cv2.putText(self.frame, class_name,bottom_right,cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_hash,1)

            self.image_pub.publish(image_to_msg(self.frame))

        img = msg_to_image(data)
        self.frame = cv2.resize(img, (224, 224))
        self.image_received_flag = 1
        #except Exception as e:
            #print(e)

    def cleanup(self):
        print("Shutting down crossroad detector")
        cv2.destroyAllWindows()


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    rospy.init_node("yolo", anonymous=True)
    Robot()
