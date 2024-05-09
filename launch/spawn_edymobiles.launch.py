from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='swisscat_simulation',
            executable='map_publisher',
            output='screen',
        ),
])
