import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    rob_loca_dir = get_package_share_directory('swisscat_simulation')
    rvizconfig = LaunchConfiguration('rvizconfig', default=os.path.join(rob_loca_dir, 'rviz', 'loca.rviz'))
    urdf_path = os.path.join(rob_loca_dir, 'urdf/edison0.urdf')
    robot_state_publisher_node = Node(
         package='robot_state_publisher',
         executable='robot_state_publisher',
         output='screen',
         parameters=[{'robot_description': open(urdf_path, 'r').read(), 'use_sim_time': 'true'}],
     )

    transforms_node = Node(
        package='swisscat_simulation',
        executable='transforms',
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        arguments=[urdf_path],
        parameters=[{'use_sim_time': 'true'}],
    )
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        output='screen',)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rvizconfig],
        output='screen',
    )

    return LaunchDescription([
        rviz_node,
        static_transform_publisher_node,
        #transforms_node,
        #joint_state_publisher_node
    ])
