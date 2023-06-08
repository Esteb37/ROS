import os
import cv2

# duplicate the "images" folder into "images_resized"
path = "/home/estebanp/catkin_ws/src/final_challenge/training/images"

for folder in os.listdir(path):
    for file in os.listdir(path + "/" + folder):
        img = cv2.imread(path + "/" + folder + "/" + file)
        if img is not None:
            img = cv2.resize(img, (640, 640))
            new_folder = path + "_resized/" + folder
            if not os.path.exists(new_folder):
                os.makedirs(new_folder)
            cv2.imwrite(new_folder + "/" + file, img)
