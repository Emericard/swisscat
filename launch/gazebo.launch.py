from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
        output='screen')
    
    
    
    return LaunchDescription([
        gazebo
    ])