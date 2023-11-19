########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1A ###########################################
# Team ID:eYRC#HB#2836
# Team Leader Name:Bipin Yadav
# Team Members Name: Viraj Patkar, Utkarsh saste, Pranay Rajak
# College: Vivekanand Education Society Institute of Technology
########################################################################################################################


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from functools import partial

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("task_1a_2836")
         # Publisher for turtle1's command velocity
        self.cmd_vel_pub=self.create_publisher(Twist,"/turtle1/cmd_vel",10)
         
         # Publisher for turtle2's command velocity
        self.cmd_vel_pub2=self.create_publisher(Twist,"/turtle2/cmd_vel",10)
        
        # Subscription to turtle1's pose
        self.pose_sub=self.create_subscription(Pose,"/turtle1/pose",self.sub_controller,10)
        
         # Subscription to turtle2's pose
        self.pose_sub2=self.create_subscription(Pose,"/turtle2/pose",self.sub_controller_2,10)
        
         # Log initialization message
        self.get_logger().info("The turtle is drawing circle")
        
        # Initialization for controlling turtle1
        self.h=0
        
        # Initialization for controlling turtle2
        self.h1=0
        
    '''
    Purpose:
    ---
    Callback function to control the movement of the first turtle to make it draw a circle.

    Input Arguments:
    ---
    `msg` :  [Pose]
        Pose information of the first turtle.

    Returns:
    ---
    None

    Example call:
    ---
    This function is called automatically when the /turtle1/pose topic is updated.
    '''
    
    # Callback function to control the movement of turtle1 to make it draw a circle.
    def sub_controller(self,msg:Pose):
        cmd=Twist()
        
        # Check if turtle1's initial orientation is reached
        if( round(msg.theta,2)==-0.01 and self.h==0):
          self.h+=1
 
        elif(self.h==1):
            self.h+=1
            self.spawn_service(msg.x,msg.y,0.0,"turtle2")

        elif(self.h==2):
          cmd.linear.x=0.0
          cmd.angular.z=0.0
          self.cmd_vel_pub.publish(cmd)


        elif self.h<1:
          cmd.linear.x=1.0
          cmd.angular.z=1.0
          self.cmd_vel_pub.publish(cmd)

    '''
    Purpose:
    ---
    Callback function to control the movement of the second turtle to make it draw a circle.

    Input Arguments:
    ---
    `msg1` :  [Pose]
        Pose information of the second turtle.

    Returns:
    ---
    None

    Example call:
    ---
    This function is called automatically when the /turtle2/pose topic is updated.
    '''
       
    # Callback function to control the movement of turtle2 to make it draw a circle.
    def sub_controller_2(self,msg1:Pose):
        cmd_vel=Twist()

        # Check if turtle2's initial orientation is reached
        if(round(msg1.theta,2)==0.01):
            self.h1+=1

        elif( self.h1==1):
            cmd_vel.linear.x=0.0
            cmd_vel.angular.z=0.0
            self.cmd_vel_pub2.publish(cmd_vel)
          
        elif self.h1<1:
            cmd_vel.linear.x=2.0
            cmd_vel.angular.z=-1.0
            self.cmd_vel_pub2.publish(cmd_vel)


    '''
    Purpose:
    ---
    Spawn a new turtle with a given position and orientation using the /spawn service.

    Input Arguments:
    ---
    `x` :  [float]
        X position where the new turtle will be spawned.

    `y` :  [float]
        Y position where the new turtle will be spawned.

    `theta` :  [float]
        Initial orientation of the new turtle.

    `name` :  [str]
        Name of the new turtle.

    Returns:
    ---
    None

    Example call:
    ---
    spawn_service(2.0, 2.0, 0.0, "turtle2")
    '''
    # Spawn a new turtle with a given position and orientation using the /spawn service.
    def spawn_service(self,x,y,theta,name):
        client=self.create_client(Spawn,"/spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("The Service is up")

        request=Spawn.Request()
        request.x=x
        request.y=y
        request.theta=theta
        request.name=name
        future=client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn))


    '''
    Purpose:
    ---
    Callback function to handle the result of the /spawn service call.

    Input Arguments:
    ---
    `future` :  [Future]
        Future object representing the result of the service call.

    Returns:
    ---
    None

    Example call:
    ---
    This function is called automatically when the service call is completed.
    '''
    
    # Callback function to handle the result of the /spawn service call.
    def callback_spawn(self,future):
        try:
          response=future.result()
        except Exception as e:
            self.get_logger().error("Service Failed %r" %(e,))

def main(args=None):
        rclpy.init(args=args)
        node=DrawCircleNode()
        rclpy.spin(node)
        rclpy.shutdown()