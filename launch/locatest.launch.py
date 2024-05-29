import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
import xacro
from launch_ros.actions import Node

def generate_launch_description():

    rob_loca_dir = get_package_share_directory('swisscat_simulation')
    map_path = os.path.join(rob_loca_dir, 'maps', 'circuit.yaml')
    nav2_params_path = os.path.join(rob_loca_dir, 'params/nav_params.yaml')
    init_poses = ['0.6', '6', '0', '1.5707',
                  '0.5', '1', '0', '0',
                  '-1.28', '3.07', '0', '1.5707',
                  '1.10', '-3.595', '0', '1.5707',
                  '1.10', '-4.548', '0', '1.5707']
    robot_names = ['Robot1', 'Robot2']
    gazebo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
           os.path.join(rob_loca_dir, 'launch', 'gazebo.launch.py')
       )
   )
    overtake_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
           os.path.join(rob_loca_dir, 'launch', 'overtake.launch.py')
       )
   )
    map2gazebo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
           os.path.join(rob_loca_dir, 'launch', 'map2gazebo.launch.py')
       )
   )
    mapserver_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
           os.path.join(rob_loca_dir, 'launch', 'mapserver.launch.py')
       )
   )
    rviz_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
           os.path.join(rob_loca_dir, 'launch', 'rviz.launch.py')
       )
   )
    launches = []
    launches.append(gazebo_launch)
    #launches.append(map2gazebo_launch)
    launches.append(mapserver_launch)
    launches.append(overtake_launch)
    launches.append(rviz_launch)

    for i in range(len(robot_names)):
        edison = 'edison' + str(i) + '.urdf'
        xacro_file = os.path.join(
        get_package_share_directory('swisscat_simulation'),
        'urdf',
        edison
        )

        robot_description = xacro.process_file(xacro_file).toxml()
        sensor_file_path = os.path.join(get_package_share_directory('swisscat_simulation'), 'launch', 'sensor.launch.py')
        sensor_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensor_file_path)
        )

        ekf_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(rob_loca_dir, 'launch'), '/ekf.launch.py'])
        )

        nav_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 'launch'), '/bringup_launch.py']),
            launch_arguments={
                'map': map_path,
                'use_sim_time': 'False',
                'params_file': nav2_params_path,
                'autostart': 'true',
            }.items()
        )
        
        # Grouping sensor, EKF, and navigation launch descriptions under the same namespace
        robot_group = GroupAction([
            PushRosNamespace(robot_names[i]),
            sensor_launch_description,
            ekf_launch_description,
            #nav_launch_description,
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
                    'robot_namespace': robot_names[i],
                }.items())
        ])

        launches.append(robot_group)

    return LaunchDescription(launches)
