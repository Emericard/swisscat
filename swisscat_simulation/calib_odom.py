#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import time

"""
Basic navigation demo to go to waypoints.
"""

# Predefined pose for each station, hard-defined here

from geometry_msgs.msg import PoseStamped

class Station:
    def __init__(self, posX, posY, orientationZ, orientationW):
        self.posX = posX
        self.posY = posY
        self.orientationZ = orientationZ
        self.orientationW = orientationW
        self.goal_pose = PoseStamped()
    def define_goal_pose(self):
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = self.posX
        self.goal_pose.pose.position.y = self.posY
        self.goal_pose.pose.orientation.z = self.orientationZ
        self.goal_pose.pose.orientation.w = self.orientationW
        return self.goal_pose

def main():
    rclpy.init()
    
    # create fixed stations
    station_OBL = Station(posX = 0.0, posY = 0.0, orientationZ = 1.0, orientationW = 0.0)
    station_OTL = Station(posX = 0.0, posY = 1.2, orientationZ = 0.7071068, orientationW = 0.7071068)
    station_OTR = Station(posX = 1.2, posY = 1.2, orientationZ = 0.0, orientationW = 1.0)
    station_OBR = Station(posX = 1.2, posY = 0.0, orientationZ = 0.7071068, orientationW = -0.7071068)

    goal_poses = []
    goal_pose1 = station_OTL.define_goal_pose()
    goal_poses.append(goal_pose1)
    goal_pose2 = station_OTR.define_goal_pose()
    goal_poses.append(goal_pose2)
    goal_pose3 = station_OBR.define_goal_pose()
    goal_poses.append(goal_pose3)
    goal_pose4 = station_OBL.define_goal_pose()
    goal_poses.append(goal_pose4)

    navigator = BasicNavigator()

    navigator._waitForNodeToActivate('bt_navigator')

    while True:
        navigator.followWaypoints(goal_poses) 

        i = 0
        while not navigator.isTaskComplete():

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                # print(feedback)
                print('Executing current waypoint: ' +
                      str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
                now = navigator.get_clock().now()
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            #     navigator.cancelNav()

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded, going to next waypoint!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            break
        elif result == TaskResult.FAILED:
            print('Goal failed, returning home!')
            break
        else:
            print('Goal has an invalid return status!')
            break

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    while True:
        main()
        time.sleep(1)
