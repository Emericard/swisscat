from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    init_pose_x = LaunchConfiguration('init_pose_x')
    init_pose_y = LaunchConfiguration('init_pose_y')
    init_pose_z = LaunchConfiguration('init_pose_z')
    init_pose_Y = LaunchConfiguration('init_pose_Y')
    robot_description = LaunchConfiguration('robot_description')
    robot_namespace = LaunchConfiguration('robot_namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot',
            description='Robot model name'
        ),
        DeclareLaunchArgument(
            'init_pose',
            default_value='',
            description='Initial position and orientation'
        ),
        DeclareLaunchArgument(
            'robot_description',
            default_value='',
            description='Robot description parameter'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot_model',
            output='screen',
            arguments=['-entity', robot_name,
                       '-topic', 'robot_description',
                       '-robot_namespace', robot_namespace,
                       '-x', init_pose_x,
                       '-y', init_pose_y,
                       '-z', init_pose_z,
                       '-Y', init_pose_Y],
            parameters=[robot_description]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
    ])