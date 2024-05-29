import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
   map_publisher = Node(
        package='swisscat_simulation',
        executable='map_publisher',
        output='screen',
    )
   
   return LaunchDescription([map_publisher])