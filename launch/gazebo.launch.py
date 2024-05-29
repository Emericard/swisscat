from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_path = get_package_share_directory('swisscat_simulation')
    world_path = os.path.join(world_path, 'worlds','map.sdf')
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_factory.so'],
        output='screen')
    
    
    
    return LaunchDescription([
        gazebo
    ])