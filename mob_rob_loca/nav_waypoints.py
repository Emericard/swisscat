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
from rclpy.duration import Duration
from mob_rob_loca.RobotPlatform import Station, Robot
import time

"""
Basic navigation demo to go to waypoints.
"""

# Predefined pose for each station, hard-defined here

def main():
    rclpy.init()
    
    # create fixed stations
    # station_NMR = Station(posX = 4.189, posY = 7.55, orientationZ = 0.0, orientationW = 1.0)
    # station_OMNI = Station(posX = 4.263, posY = 0.454, orientationZ = 0.0, orientationW = 1.0)
    station_UTL = Station(posX = 5.0, posY = 2.0, orientationZ = 0.0, orientationW = 0.0)
    station_UBL = Station(posX = 7.0, posY = 2.0, orientationZ = 0.0, orientationW = 0.0)
    station_UBR = Station(posX = 5.0, posY = 2.0, orientationZ = 0.0, orientationW = 0.0)
    # station_UBR2 = Station(posX = 4.32, posY = -0.36, orientationZ = -0.702225, orientationW = 0.711955)
    station_UTR = Station(posX = 7.0, posY = 2.0, orientationZ = 0.0, orientationW = 0.0)
    # station_HOME = Station(posX = 4.2, posY = 2.0, orientationZ = 1.0, orientationW = 0.0) #random values here

    # create robot
    robot1 = Robot(robot_type=Robot.Type.TRANSPORTER, state=Robot.State.STANDBY, goals=[], empty=True, current_pose=PoseStamped())

    goal_poses = []
    goal_pose1 = station_UTL.define_goal_pose()
    goal_poses.append(goal_pose1)
    goal_pose2 = station_UBL.define_goal_pose()
    goal_poses.append(goal_pose2)
    goal_pose3 = station_UBR.define_goal_pose()
    goal_poses.append(goal_pose3)
    goal_pose4 = station_UTR.define_goal_pose()
    goal_poses.append(goal_pose4)

    # if robot1.state == Robot.State.STANDBY:
    #     print(goal_poses)
    #     robot1.goals = goal_poses
    # else:
    #     print("Robot is not in STANDBY state. Cannot assign goals.")

    navigator = BasicNavigator()

    navigator._waitForNodeToActivate('bt_navigator')

    while True:
        navigator.followWaypoints(goal_poses) # check for namespaces to do this

        i = 0
        while not navigator.isTaskComplete():

            # Do something with the feedback 
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                # print(feedback) # nav2_msgs.action.FollowWaypoints_Feedback(current_waypoint=0)
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
        time.sleep(10)
