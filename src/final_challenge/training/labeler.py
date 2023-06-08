import os
import cv2
import numpy as np
import time

path = "/home/estebanp/catkin_ws/src/final_challenge/training/images_resized"


top_right = None
bottom_left = None
temp_bottom_left = None

def drag_event(event, x, y, flags, param):
	global top_right, bottom_left, temp_bottom_left
	if event == cv2.EVENT_LBUTTONDOWN:
		top_right = (x, y)
	# while draggin
	elif event == cv2.EVENT_MOUSEMOVE and top_right is not None:
		temp_bottom_left = (x, y)
	# when draggin stops
	elif event == cv2.EVENT_LBUTTONUP:
		bottom_left = (x, y)

cv2.namedWindow("image")
cv2.setMouseCallback("image", drag_event)

for i, folder in enumerate(os.listdir(path)):
    for file in os.listdir(path + "/" + folder):
        name = path + "/" + folder + "/" + file
		# check if there is a label already

        if os.path.isfile("/home/estebanp/catkin_ws/src/final_challenge/training/labels/" + folder + "/" + file[:-4] + ".txt"):
            continue

        img = cv2.imread(name)

        if img is not None:
            while top_right is None or bottom_left is None:
                copy = img.copy()
                if top_right is not None and temp_bottom_left is not None:
                    cv2.rectangle(copy, top_right, temp_bottom_left, (0, 255, 0), 2)
                cv2.imshow("image", copy)
                cv2.waitKey(1)
        center_x = int((top_right[0] + bottom_left[0]) / 2)
        center_y = int((top_right[1] + bottom_left[1]) / 2)
        width = abs(top_right[0] - bottom_left[0])
        height = abs(top_right[1] - bottom_left[1])

        with open("/home/estebanp/catkin_ws/src/final_challenge/training/labels/" + folder + "/" + file[:-4] + ".txt", "w") as f:
            f.write(str(i) + " " + str(center_x) + " " + str(center_y) + " " + str(width) + " " + str(height) + "\n")

        top_right = None
        bottom_left = None
        temp_bottom_left = None