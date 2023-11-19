#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		[ 2836 ]
# Author List:		[ Bipin Yadav , Pranay Rajak, Utkatsh Saste , Viraj Patkar]
# Filename:		controller.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		subscribing to detected aruco publishing to wrench topic 

################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
import numpy as np
import time
import math
from tf_transformations import euler_from_quaternion

from my_robot_interfaces.srv import NextGoal             

# You can add more if required
##############################################################


# Initialize Global variables


################# ADD UTILITY FUNCTIONS HERE #################

##############################################################


# Define the HBController class, which is a ROS node
class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        self.sub=self.create_subscription(Pose2D,"/detected_aruco",self.feedback,10)
        self.puba=self.create_publisher(Wrench,"/hb_bot_1/left_wheel_force",10)
        self.pubb=self.create_publisher(Wrench,"/hb_bot_1/right_wheel_force",10)
        self.pubc=self.create_publisher(Wrench,"/hb_bot_1/rear_wheel_force",10)
        
        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	    #	Use the below given topics to generate motion for the robot.
	    #   /hb_bot_1/left_wheel_force,
	    #   /hb_bot_1/right_wheel_force,
	    #   /hb_bot_1/left_wheel_force




        # For maintaining control loop rate.
        self.rate = self.create_rate(100)


        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0
        self.kpx=1
        self.kpy=1
        self.kptheta=0.3

    
    # Method to create a request to the "next_goal" service
    def send_request(self, request_goal):
        self.req.request_goal = request_goal
        self.future = self.cli.call_async(self.req)
        
    def feedback(self,msg:Pose2D):
        global hb_x,hb_y,hb_theta
        hb_x=msg.x
        hb_y=msg.y
        hb_theta=msg.theta
        

    def inverse_kinematics(self,vx,vy,w):
        ############ ADD YOUR COE HERE ############
        
        len1=0.68
        rad=0.14
        sp1=math.sqrt(3)
        sp2=1/2
       
        
        RotMat=np.array([[-1/rad,1/rad,0/rad],[-1/rad,-1/(2*rad),-sp1/(2*rad)],[-1/rad,-1/(2*rad),sp1/(2*rad)]])
        variables=np.array([[w],[vx],[vy]])
       
        result=np.dot(RotMat,variables)
        v3=result[0][0]
        v2=result[1][0]
        v1=result[2][0]
        return v1,v2,v3


        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################
        pass


def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the HBController class
    hb_controller = HBController()
   
    # Send an initial request with the index from ebot_controller.index
    hb_controller.send_request(hb_controller.index)
    
    # Main loop
    while rclpy.ok():

        # Check if the service call is done
        if hb_controller.future.done():
            try:
                # response from the service call
                response = hb_controller.future.result()
            except Exception as e:
                hb_controller.get_logger().infselfo(
                    'Service call failed %r' % (e,))
            else:
                #########           GOAL POSE             #########
                x_goal      = response.x_goal+250
                y_goal      = response.y_goal+250
                theta_goal  = response.theta_goal
                hb_controller.flag = response.end_of_list
                ####################################################
                
                # Calculate Error from feedback
                errorX=x_goal-hb_x
                errorY=y_goal-hb_y
                errorTheta=theta_goal-hb_theta

                # print(abs(errorX),abs(errorY),errorTheta)
                # Change the frame by using Rotation Matrix (If you find it required)
                vx=errorX*hb_controller.kpx
                vy=errorY*hb_controller.kpy
                w=errorY*hb_controller.kptheta
                # Calculate the required velocity of bot for the next iteration(s)
                forces=hb_controller.inverse_kinematics(vx,vy,w)
                # Find the required force vectors for individual wheels from it.(Inverse Kinematics)
                pub1=Wrench()
                pub2=Wrench()
                pub3=Wrench()
                pub1.force.y=forces[0]
                pub2.force.y=forces[1]
                pub3.force.y=forces[2]
                hb_controller.puba.publish(pub1)
                hb_controller.pubb.publish(pub2)
                hb_controller.pubc.publish(pub3)
                # Apply appropriate force vectors

                # Modify the condition to Switch to Next goal (given position in pixels instead of meters)
                if abs(errorX)<=2 and abs(errorY)<=2 and  abs(errorTheta)<=0.3:
                ############     DO NOT MODIFY THIS       #########
                  hb_controller.index += 1
                  print(hb_controller.index)
                  print(errorTheta)
                  if hb_controller.flag == 1 :
                      hb_controller.index = 0
                  hb_controller.send_request(hb_controller.index)
                ####################################################

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
