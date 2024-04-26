from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='$(find swisscat_simulation)/config/map2gazebo_defaults.yaml',
        description='Path to the parameters file'
    )

    export_dir_arg = DeclareLaunchArgument(
        'export_dir',
        default_value='$(find swisscat_simulation)/models/map/meshes',
        description='Export directory path'
    )

    map2gazebo_node = Node(
        package='swisscat_simulation',
        executable='map2gazebo',
        output='screen',
        parameters=[
            {'export_dir': LaunchConfiguration('export_dir')}
        ],
        remappings=[
            ('__params', LaunchConfiguration('params_file'))
        ]
    )

    return LaunchDescription([
        params_file_arg,
        export_dir_arg,
        map2gazebo_node
    ])

if __name__ == '__main__':
    generate_launch_description()
