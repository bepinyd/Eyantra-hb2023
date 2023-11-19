########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1B ###########################################
# Team ID:2836
# Team Leader Name:Bipin Yadav
# Team Members Name: Viraj Patkar, Utkarsh saste, Pranay Rajak
# College: Vivekanand Education Society Institute of Technology
########################################################################################################################

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal

# ROS Node for controlling the robot
class HBTask1BController(Node):

   def __init__(self):
        super().__init__('hb_task1b_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pose_sub = self.create_subscription(Odometry, "/odom", self.odometryCb, 10)
        self.vel = Twist()
        self.rate = self.create_rate(100)
        self.hb_x = 0
        self.cli = self.create_client(NextGoal, 'next_goal')
        self.req = NextGoal.Request()
        self.index = 0
        self.px = 1  # Proportional gain for linear x velocity
        self.py = 1  # Proportional gain for linear y velocity
        self.ptheta = 0.3  # Proportional gain for angular velocity

   # Callback function for receiving odometry data
   def odometryCb(self, msg: Odometry):
        global hb_x, hb_y, hb_theta
        hb_x = msg.pose.pose.position.x
        hb_y = msg.pose.pose.position.y
        hb_theta = msg.pose.pose.orientation.z

   # Send a request to the 'next_goal' service
   def send_request(self, index):
        self.req.request_goal = self.index
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    
    ebot_controller = HBTask1BController()
    ebot_controller.send_request(ebot_controller.index)
    
    while rclpy.ok():

        if ebot_controller.future.done():
            try:
                response = ebot_controller.future.result()
            except Exception as e:
                ebot_controller.get_logger().info('Service call failed %r' % (e,))
            else:
                # Extract goal pose information
                x_goal = response.x_goal
                y_goal = response.y_goal
                theta_goal = response.theta_goal
                ebot_controller.flag = response.end_of_list
                
                # Compute errors in position and orientation
                error_x = x_goal - hb_x
                error_y = y_goal - hb_y
                error_theta = theta_goal - hb_theta
                print(error_theta,hb_theta)

                # Adjust velocities based on proportional control
                ebot_controller.vel.linear.x = 0.0
                ebot_controller.vel.linear.y = 0.0
                ebot_controller.vel.angular.z = 0.0
                ebot_controller.cmd_vel_pub.publish(ebot_controller.vel)

                # Check if the robot has reached the goal
                if error_x < 0.1 and error_y < 0.1 and error_theta < 0.1:
                    ebot_controller.index += 1

                    # Reset the index if the end of the list is reached
                    if ebot_controller.flag == 1:
                        ebot_controller.index = 0

                    ebot_controller.send_request(ebot_controller.index)

        rclpy.spin_once(ebot_controller)

    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
        main()
