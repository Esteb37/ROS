# Importando las librerias del dron
from djitellopy import Tello
import cv2
import numpy as np

MODE = "WEBCAM"  # "WEBCAM" or "DRONE"

H_min = 0  # HUE (0-180) = (0-360)
S_min = 21  # SATURATION (0-255)
V_min = 173  # VALUE (0-255)

H_max = 255
S_max = 123
V_max = 255


# Connecting to the drone
drone = Tello()
capture = None

if MODE == "DRONE":
    drone.connect()
    drone.streamoff()
    drone.streamon()
else:
    capture = cv2.VideoCapture(0)

# Drone speeds initialize
drone.left_right_velocity = 0
drone.for_back_velocity = 0
drone.up_down_velocity = 0
drone.yaw_velocity = 0


def main():
    print("Main program inicialized")
    while True:

        img = None

        if MODE == "DRONE":
            frame_read = drone.get_frame_read()
            img = frame_read.frame
        else:
            ret, img = capture.read()

        img = cv2.resize(img, (500, 500))
        img = cv2.flip(img, 1)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, (H_min, S_min, V_min), (H_max, S_max, V_max))
        
        cv2.imshow("Image", mask)

        if MODE == "DRONE":
            drone.send_rc_control(drone.left_right_velocity,
                                  drone.for_back_velocity,
                                  drone.up_down_velocity,
                                  drone.yaw_velocity)

        if (cv2.waitKey(1) & 0xFF == ord('l')):
            cv2.destroyAllWindows()

try:
    main()

except KeyboardInterrupt:
    print('KeyboardInterrupt exception is caught')
    cv2.destroyAllWindows()

    if MODE == "DRONE":
        print("Landing")
        drone.land()
        drone.streamoff()

else:
    print('No exceptions are caught')
