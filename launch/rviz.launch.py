import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    rob_loca_dir = get_package_share_directory('swisscat_simulation')
    urdf_path = os.path.join(rob_loca_dir, 'urdf/edison.urdf')
    rvizconfig = LaunchConfiguration('rvizconfig', default=os.path.join(rob_loca_dir, 'rviz', 'loca.rviz'))

    transforms_node = Node(
        package='swisscat_simulation',
        executable='transforms',
        name='transforms',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rvizconfig],
        output='screen',
        remappings=[('/odom', '/odometry/filtered')],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': open(urdf_path, 'r').read()}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        arguments=[urdf_path],
    )

    
    
    return LaunchDescription([
        transforms_node,
        rviz_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
