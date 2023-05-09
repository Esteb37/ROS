#!/usr/bin/env python
"""
    Agregar:
        Callback a color de luz de semaforo (red, green, yellow, none)
        Suscrito a tópico de /traffic_light
    
        Reglas de control de velocidad según luz
            Si se está moviendo y detecta verde o nada, seguir igual
            Si se está moviendo y detecta amarillo, reducir la velocidad al 50%
            Si se está moviendo y detecta rojo, detenerse por completo
            
            Si llega a un goal, dejar de avanzar por completo hasta que vea verde
            Si llega a un goal y no ve nada, ve rojo, o ve amarillo, detenerse
            Si está en un goal y ve verde, avanzar
            
            Si está detenido por la luz roja y ve verde, avanzar
            Si está detenido por la luz roja y ve rojo, amarillo o nada, seguir detenido
"""


import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


class PathGenerator():
    def __init__(self):

        # Initialize ROS node
        rospy.on_shutdown(self.cleanup)
        r = rospy.Rate(50)  # 50 Hz

        # Publishers and subscribers
        self.path_goal = rospy.Publisher('/goal', Pose, queue_size=1)
        rospy.Subscriber('/reached_goal', Bool, self.bandera_cb)

        goal = Pose()
        path = rospy.get_param("/path")
        i = 0
        self.flag = True

        while not rospy.is_shutdown():

            if i >= len(path):
                print("Path completed")
                break

            if (self.flag == True):
                goal.position.x = path[i][0]
                goal.position.y = path[i][1]
                i += 1
                self.flag = False
                print("Goal: ", goal.position.x, goal.position.y)

            self.path_goal.publish(goal)
            r.sleep()

    def bandera_cb(self, msg):
        self.flag = msg.data

    def cleanup(self):
        print("Path generator node terminated.")


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    rospy.init_node("path_generator", anonymous=True)
    PathGenerator()
