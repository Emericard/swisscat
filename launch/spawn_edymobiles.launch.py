from launch import LaunchDescription
from launch.actions import  DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro

def generate_launch_description():
    
    xacro_file = os.path.join(
        get_package_share_directory('swisscat_simulation'),
        'urdf',
        'edison.urdf'
    )

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()  # Process to XML and store as string
    init_poses = ['-3.18', '-3.595', '0', '1.5707',
                  '-3.18', '-4.548', '0', '1.5707',
                  '-1.28', '3.07', '0', '1.5707',
                  '1.10', '-3.595', '0', '1.5707',
                  '1.10', '-4.548', '0', '1.5707']
    robot_names = ['Robot1', 'Robot2', 'Robot3', 'Robot4', 'Robot5']

    spawn_robot_nodes = []
    for i in range(2):
        spawn_robot_nodes.append(GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('swisscat_simulation'),
                                               '/launch/',
                                               'one_robot_with_name.launch.py']),
                launch_arguments={
                    'init_pose_x': init_poses[i*4 + 0],
                    'init_pose_y': init_poses[i*4 + 1],
                    'init_pose_z': init_poses[i*4 + 2],
                    'init_pose_Y': init_poses[i*4 + 3],
                    'robot_name': robot_names[i],
                    'robot_description': robot_description,
                    'robot_namespace': robot_names[i]
                }.items()
            )
        ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'robotNamespace1',
            default_value='robot1',
            description='Robot 1 namespace'
        ),
        DeclareLaunchArgument(
            'robotNamespace2',
            default_value='robot2',
            description='Robot 2 namespace'
        ),
        DeclareLaunchArgument(
            'robotNamespace3',
            default_value='robot3',
            description='Robot 3 namespace'
        ),
        DeclareLaunchArgument(
            'robotNamespace4',
            default_value='robot4',
            description='Robot 4 namespace'
        ),
        DeclareLaunchArgument(
            'robotNamespace5',
            default_value='robot5',
            description='Robot 5 namespace'
        ),
        *spawn_robot_nodes
    ])
