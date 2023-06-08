import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import numpy as np
import time
import cv2
import PIL.Image as Image
import matplotlib.pylab as plt

import tensorflow as tf
import tensorflow_hub as hub


def load_img(path):
  img = tf.io.read_file(path)
  img = tf.image.decode_jpeg(img, channels=3)
  return img


mobilenet_v2 ="https://tfhub.dev/google/openimages_v4/ssd/mobilenet_v2/1"

IMAGE_SHAPE = (640, 640)

detector = tf.keras.Sequential([
    hub.KerasLayer(mobilenet_v2, input_shape=IMAGE_SHAPE+(3,), output_key="detection_boxes"),
])

img = load_img("/home/estebanp/catkin_ws/src/final_challenge/training/images_resized/forward/image_0000.jpg")

# transfer learning
boxes = detector.predict(np.random.uniform(0, 1, (1, 640, 640, 3)))

# image to numpy
img = img.numpy()

# draw bounding boxes
for box in boxes:
    ymin, xmin, ymax, xmax = tuple(box)
    cv2.rectangle(img, (int(xmin*640), int(ymin*640)), (int(xmax*640), int(ymax*640)), (255, 0, 0), 2)
cv2.imshow("image", img)
cv2.waitKey(0)