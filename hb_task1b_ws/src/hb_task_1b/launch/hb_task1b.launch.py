from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node
def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('hb_task_1b'), 'urdf', 'hb_bot.urdf.xacro')
    robot_description = ParameterValue(Command (['xacro ', urdf_path]), value_type=str)
    controller_node = Node(
        package="hb_task_1b",           # Enter the name of your ROS2 package
        executable="controller",    # Enter the name of your executable
    )
    service_node = Node(
        package="hb_task_1b",           # Enter the name of your ROS2 package
        executable="service_node",   # Enter the name of your executable
    )
    return LaunchDescription([ 
      controller_node,
      service_node,
     ])
