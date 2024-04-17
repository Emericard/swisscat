import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
    rob_loca_dir = get_package_share_directory('swisscat_simulation')
    urdf_path = os.path.join(rob_loca_dir, 'urdf/edison.urdf')
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': open(urdf_path, 'r').read()}]
    )
    transforms_node = Node(
        package='swisscat_simulation',
        executable='transforms',
        name='transforms',
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        arguments=[urdf_path],
    )
    
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
        output='screen')
    
    
    
    return LaunchDescription([
        # transforms_node,
        # robot_state_publisher_node,
        # joint_state_publisher_node,
        gazebo
    ])