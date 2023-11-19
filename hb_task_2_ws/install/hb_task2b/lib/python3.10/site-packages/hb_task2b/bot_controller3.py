#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2B of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		[ 2836 ]
# Author List:		[ BIPIN YADAV , PRANAY RAJAK , VIRAJ PATKAR , UTKARSH SASTE ]
# Filename:		bot_controller.py
# Functions:
#			[ inverse kinematics, goal callback , feedback , main ]
# Nodes:		hb_bot_3/left right and rear wheels and detected/aruco_3


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Goal             
from geometry_msgs.msg import Pose2D
import numpy as np
from geometry_msgs.msg import Wrench

class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        

        # Initialise the required variables
        self.bot_3_x = []
        self.bot_3_y = []
        self.bot_3_theta = 0.0
        self.index=0
        self.hb_x=0.0
        self.hb_y=0.0
        self.hb_theta=0.0
        self.px=0.6
        self.py=0.6
        self.botid=None
        self.ptheta=0.1
        self.goal_flag=True
        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	    #	Use the below given topics to generate motion for the robot.
	    #   /hb_bot_3/left_wheel_force,
	    #   /hb_bot_3/right_wheel_force,
	    #   /hb_bot_3/left_wheel_force
        self.feedbacksub=self.create_subscription(Pose2D,'/detected_aruco_3',self.feedback,10)
        self.puba=self.create_publisher(Wrench,"/hb_bot_3/left_wheel_force",10)
        self.pubb=self.create_publisher(Wrench,"/hb_bot_3/right_wheel_force",10)
        self.pubc=self.create_publisher(Wrench,"/hb_bot_3/rear_wheel_force",10)

        #Similar to this you can create subscribers for hb_bot_3 and hb_bot_3
        self.subscription = self.create_subscription(
            Goal,  
            'hb_bot_3/goal',  
            self.goalCallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )  

        # self.subscription  # Prevent unused variable warning

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

    def inverse_kinematics(self, vx, vy ,w):
        ############ ADD YOUR CODE HERE ############
          
           
        len1=0.68
        rad=0.14
        sp1=math.sqrt(3)
        sp2=1/2
        sp3=1/math.sqrt(3)
       
        
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

    def goalCallBack(self, msg):
      if self.goal_flag:
        print('done1')
        self.bot_3_x = msg.x
        self.bot_3_y = msg.y
        self.botid=msg.bot_id
        self.goal_flag=False

    def feedback(self,msg:Pose2D):
        self.hb_x=msg.x
        self.hb_y=msg.y
        self.hb_theta=0.0
        

def main(args=None):
    rclpy.init(args=args)
    
    hb_controller = HBController()
    
    # Main loop
    while rclpy.ok():
            if hb_controller.goal_flag:
              time.sleep(0.4)
            hb_controller.feedback
            
            rclpy.spin_once(hb_controller)

            if not hb_controller.goal_flag:
               error_x=hb_controller.bot_3_x[hb_controller.index]-hb_controller.hb_x
               error_y=hb_controller.bot_3_y[hb_controller.index]-hb_controller.hb_y
               error_theta=hb_controller.bot_3_theta-hb_controller.hb_theta
               vx=error_x*hb_controller.px
               vy=error_y*hb_controller.py
               vtheta=error_theta*hb_controller.ptheta
           
               forces=hb_controller.inverse_kinematics(vx,vy,vtheta)
               pub1=Wrench()
               pub2=Wrench()
               pub3=Wrench()
               pub1.force.y=forces[0]
               pub2.force.y=forces[1]
               pub3.force.y=forces[2]
               hb_controller.puba.publish(pub1)
               hb_controller.pubb.publish(pub2)
               hb_controller.pubc.publish(pub3)


               if hb_controller.index==len(hb_controller.bot_3_x)-1:
                   hb_controller.index=0
                   hb_controller.get_logger().info('bot 3 finished ')

            
               elif abs(error_x)<=2 and abs(error_y)<=2 and hb_controller.index<len(hb_controller.bot_3_x):
                   hb_controller.index+=1
                   print(hb_controller.index)
               

                

        # Spin once to process callbacks
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
