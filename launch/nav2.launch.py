from argparse import Namespace
from distutils.cmd import Command
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from launch_ros.substitutions import FindPackageShare
import launch
import yaml
import xacro

def generate_launch_description():
    rob_loca_dir = get_package_share_directory('swisscat_simulation')
    map_path = os.path.join(rob_loca_dir, 'maps/circuit.yaml')
    nav2_params_path = os.path.join(rob_loca_dir, 'params/nav_params.yaml')
    map_path = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('swisscat_simulation'),
            'maps',
            'circuit.yaml'
        )
    )

    namespace=LaunchConfiguration('namespace')
    rviz_config = LaunchConfiguration('rviz_config')

    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )
    xacro_file = os.path.join(
        get_package_share_directory('swisscat_simulation'),
        'urdf',
        'edison.urdf'
    )

    robot_description = xacro.process_file(xacro_file).toxml()


    namespaced_params= ReplaceString(
        source_file=nav2_params_path, replacements={"/namespace":("/",namespace)}
    )

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config, replacements={"/tb2": ("/", namespace)})

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_path,
            description='Full path to map file to load'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_path,
            description='Full path to nav2 param file to load'
        ),

        DeclareLaunchArgument(
            'namespace',
            default_value='tb2',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_namespace',
            default_value='true',
            description='Whether to apply a namespace to the navigation stack'),

        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                rob_loca_dir, 
                'rviz', 
                'loca.rviz'),
            description='Full path to the RVIZ config file to use'),

        DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='open rviz'),
        

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            remappings=remappings,
            parameters=[{'robot_description':robot_description}]
        ),
        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_path,
                'params_file': namespaced_params}.items(),
        ),

        Node(
            condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration("open_rviz")),
            package='rviz2',
            executable='rviz2',
            namespace=namespace,
            arguments=['-d', namespaced_rviz_config_file],
            output='screen',
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ('/goal_pose', 'goal_pose'),
                        ('/clicked_point', 'clicked_point'),
                        ('/initialpose', 'initialpose')]),  
    ])