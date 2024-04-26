import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
   rob_loca_dir = get_package_share_directory('swisscat_simulation')

   sensor_conv_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swisscat_simulation'), 'launch'),
         '/sensor.launch.py'])
      )
   rviz_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swisscat_simulation'), 'launch'),
         '/rviz.launch.py'])
      )
   ekf_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swisscat_simulation'), 'launch'),
         '/ekf.launch.py'])
      )
   map2gazebo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swisscat_simulation'), 'launch'),
         '/map2gazebo.launch.py'])
      )
   nav_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swisscat_simulation'), 'launch'),
         '/nav2.launch.py'])
   )
   gazebo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
           os.path.join(rob_loca_dir, 'launch', 'gazebo.launch.py')
       )
   )

   spawn_edymobiles_launch = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           os.path.join(rob_loca_dir, 'launch', 'spawn_edymobiles.launch.py')
       )
   )

   
   return LaunchDescription([
      rviz_launch,
      spawn_edymobiles_launch,
      sensor_conv_launch,
      map2gazebo_launch,
      ekf_launch,
      nav_launch,
      gazebo_launch,

   ])
