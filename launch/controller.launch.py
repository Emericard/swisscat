from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_namespaces = ['Robot1', 'Robot2', 'Robot3', 'Robot4', 'Robot5']

    # Path to the Python executable in your venv
    python_executable_path = f"{get_package_share_directory('swisscat_robot')}/swisscat_robot/venv/bin/python3"

    controller_nodes = [
        Node(
            package='swisscat_robot',
            # Use the Python executable directly to run your nodes
            executable="controller",
            name='controller',
            output='screen',
            namespace=namespace,
            #parameters=[f"{get_package_share_directory('swisscat_robot')}/config/joints.yaml"],
        ) for namespace in robot_namespaces
    ]

    return LaunchDescription([
        # Map server node
        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[
                {
                    'yaml_filename': f"{get_package_share_directory('swisscat_robot')}/maps/circuit_MAPF.yaml",
                    'frame_id': 'map'
                }
            ]
        ),
        # Fleet manager node
        Node(
            package='swisscat_robot',
            # Use the Python executable directly to run your nodes
            executable="fleet_manager",
            name='fleet_manager',
            output='screen',
        ),
    ] + controller_nodes)
