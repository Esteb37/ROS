#!/usr/bin/env python 
import rospy 
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32, Bool
import numpy as np 

class MoveClass(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) #This function will be called before killing the node.
        ######### PUBLISHERS AND SUBSCRIBERS #################
        self.path_goal = rospy.Publisher('goal', Pose, queue_size=1) 
        rospy.Subscriber('reached_goal', Bool, self.bandera_cb)
        
        ############ CONSTANTS AND VARIABLES ################
        goal = Pose()
        path = rospy.get_param("/path")
        i = 0
        self.flag = False
    
        r = rospy.Rate(20) #20 Hz

        while not rospy.is_shutdown(): 
            
            if (self.flag == True):
                goal.x = path[i][0]
                goal.y = path[i][1]
                self.i += 1
                self.flag =  False
            
            self.path_goal.publish(goal) #publish the message
            r.sleep()  #It is very important that the r.sleep function is called at least once every cycle. 
    
    def bandera_cb(self, msg):
        self.flag = msg.data
        print("callback flag")

    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        print("I'm dying, bye bye!!!") 
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)


############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("move_to_point_open_loop", anonymous=True) 
    MoveClass() 
