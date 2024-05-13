# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    package_name = 'swisscat_simulation'
    params_path = 'params/ekf_em.yaml'

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    robot_localization_file_path = os.path.join(pkg_share, params_path)

    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[robot_localization_file_path],
            remappings=[('/odometry/filtered', 'odom')],
        ),
])
