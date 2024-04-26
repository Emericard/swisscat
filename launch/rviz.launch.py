import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    rob_loca_dir = get_package_share_directory('swisscat_simulation')
    rvizconfig = LaunchConfiguration('rvizconfig', default=os.path.join(rob_loca_dir, 'rviz', 'loca.rviz'))
    urdf_path = os.path.join(rob_loca_dir, 'urdf/edison.urdf')
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': open(urdf_path, 'r').read()}],
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
        # parameters=[{'use_sim_time': 'true'}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rvizconfig],
        output='screen',
        remappings=[('/odom', '/odometry/filtered')],
    )

    return LaunchDescription([
        rviz_node,
        robot_state_publisher_node,
        transforms_node,
        joint_state_publisher_node
    ])
