import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.turtle1_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.turtle1_pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, 10)
        self.turtle1_name = 'turtle1'
        self.turtle1_alive = True
        self.turtle1_initial_y = 0.0
        self.turtle1_circle_completed = False

        self.turtle2_cmd_vel = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.turtle2_name = 'turtle2'
        self.turtle2_alive = False
        self.turtle2_initial_x = 0.0
        self.time=0
        self.kell=False

    def turtle1_pose_callback(self, msg:Pose):
        
        if not self.kell:
            self.turtle1_initial_y=msg.y
            self.kell=True
            self.get_logger().info("ran on time")
        self.get_logger().info(str(msg.y))
        self.get_logger().info(str((self.turtle1_initial_y)))

            
        if self.turtle1_alive:
            if self.time<3:
                # Make the first turtle complete a circle
                cmd_vel = Twist()
                cmd_vel.linear.x = 1.0
                cmd_vel.angular.z = 1.0
                self.turtle1_cmd_vel.publish(cmd_vel)
         
                # Check if turtle1 has completed a circle
                if abs(round(msg.y,3) ==round(self.turtle1_initial_y,3)):
                    self.time+=1
                    
            else:
                # Kill turtle1 after completing the circle
                self.kill_turtle(self.turtle1_name)
                self.turtle1_alive = False
                self.spawn_turtle2(msg.x, msg.y)

    def kill_turtle(self, name):
        kill_client = self.create_client(Kill, '/kill')
        while not kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /kill service...')
        request = Kill.Request()
        request.name = name
        future = kill_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def spawn_turtle2(self, x, y):
        spawn_client = self.create_client(Spawn, '/spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.name = self.turtle2_name
        request.theta = 0.0  # Start with 0 orientation
        future = spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Spawned ' + self.turtle2_name)
            self.turtle2_alive = True
            self.turtle2_initial_x = x
            self.control_turtle2()

    def control_turtle2(self):
        rate = self.create_rate(10)
        while self.turtle2_alive and not self.is_shutdown():
            cmd_vel = Twist()
            cmd_vel.linear.x = 1.0
            cmd_vel.angular.z = -1.0  # Rotate in the opposite direction
            self.turtle2_cmd_vel.publish(cmd_vel)
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()
