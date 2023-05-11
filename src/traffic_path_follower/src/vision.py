"""
	Nodo de ROS
	Publica a /traffic_light
 
	Suscrito a la cámara del robot, creo que está en "camera/image_raw" (para las pruebas se puede usar la webcam)
 
 	Procesamiento:
		Recibe la imagen
		Filtra ruido con un filtro gaussiano o cualquier otro que nos haya enseñado josué
		Aplica máscaras para cada color (rojo, verde, amarillo)
		Detecta si alguna de las máscaras genera un círculo mayor a un radio específico (esto lo explica manchester)
	
		Si no detecta ningún círculo o ninguno del tamaño correcto, enviar none
		Si alguna de las máscaras detecta un círculo del tamaño correcto, enviar el color correspondiente
		Si detecta más de un círculo del tamaño correcto, enviar el color del círculo más grande

	Publicar a /traffic_light
 
	Obtener rangos de HSV para cada color (rojo, verde, amarillo) de un archivo de parámetros
	Obtener un radio mínimo para considerar el semáforo como válido de un archivo de parámetros
"""

#!/usr/bin/env python 

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

min_green = np.array([50, 220, 0])
max_green = np.array([60, 255, 255])

min_red = np.array([0, 50, 50])
max_red = np.array([10, 255, 255])

min_yellow = np.array([10, 220, 0])
max_yellow = np.array([30, 255, 255])

class ImageProcessing(object):

    def __init__(self):
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.camera_callback)
        self.segmented_image_pub = rospy.Publisher("traffic_light", Image)
        self.bridge_object = CvBridge()
        self.image_received = 0 #Flag to indicate that we have already received an image
        r = rospy.Rate(10) #10Hz

        rospy.on_shutdown(self.cleanup)
        while not rospy.is_shutdown():        
            if self.image_received:
                hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

                mask_g = cv2.inRange(hsv, min_green, max_green)
                mask_r = cv2.inRange(hsv, min_red, max_red)
                mask_y = cv2.inRange(hsv, min_yellow, max_yellow)
                
                res_g = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask_g)
                res_r = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask_r)
                res_y = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask_y)

                cv2.imshow('original image',self.cv_image)
                cv2.imshow('Green', res_g)
                cv2.imshow('Red', res_r)
                cv2.imshow('Yellow', res_y)
                
                #imgs = np.concatenate((res_g, res_r, res_y), axis=0)
                #cv2.imshow('G, R, Y', imgs)

                image_message = self.bridge_object.cv2_to_imgmsg(res_y, encoding="bgr8")
                self.segmented_image_pub.publish(image_message)
            cv2.waitKey(1)
            r.sleep()
        cv2.destroyAllWindows()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.image_received=1

    def cleanup(self):
        """
        Stops the robot when the program is interrupted.
        """
        cv2.imwrite('robot_image.jpg',self.cv_image)
        print("I'm saving the last image")

if __name__ == '__main__':
    rospy.init_node('vision', anonymous=True)
    showing_image_object = ImageProcessing()