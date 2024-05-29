

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swisscat_simulation',
            executable='overtake_node',
            name='overtake_node',
            output='screen',
        )
    ])

if __name__ == '__main__':
    generate_launch_description()