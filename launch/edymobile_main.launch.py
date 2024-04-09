from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo_ros_share_path = get_package_share_directory('gazebo_ros')
    swisscat_robot_share_path = get_package_share_directory('swisscat_robot')


    params = [{'use_sim_time': use_sim_time}]

    world_path = os.path.join(swisscat_robot_share_path, 'worlds', 'circuit_MAPF_VF.world')

    # Start Gazebo server and client
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so',
             world_path],
        output='screen')

    spawn_edymobiles_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(swisscat_robot_share_path, 'launch', 'spawn_edymobiles.launch.py')
        )
    )

    ld = LaunchDescription([
        gazebo,
        spawn_edymobiles_launch
    ])

    return ld
