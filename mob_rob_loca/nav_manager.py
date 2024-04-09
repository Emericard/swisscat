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

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from .submodules.Goal_Manager import Goal_Manager
import logging
import time

"""
Basic navigation demo to go to waypoints.
"""

logger = logging.getLogger('ROBOT_MANAGER')

def main():
    rclpy.init()

    go_to_omni = True
    nmr_man = Goal_Manager('NMR')
    omni_man = Goal_Manager('OMNI')
    print('init')
    navigator = BasicNavigator()

    navigator._waitForNodeToActivate('bt_navigator')

    while(1):
        print('while')

        if go_to_omni == True:
            goal_pose = omni_man.goal_pose 
            nmr_man.calibrate_pose_pipeline()
        else:
            goal_pose = nmr_man.goal_pose
            omni_man.calibrate_pose_pipeline()
        time.sleep(2)
        goal_pose = []
        goal_pose.append(omni_man.goal_pose)
        goal_pose.append(nmr_man.goal_pose)
        #exit()
        for i in range(len(goal_pose)):
            navigator.goToPose(goal_pose[i])
            print('loop', i)
            print('goal_pose: x=%f, y=%f', goal_pose[i].pose.position.x, goal_pose[i].pose.position.y)
            j = 0

            while not navigator.isTaskComplete():
                # Do something with the feedback
                j = j + 1
                feedback = navigator.getFeedback()
                #if feedback and j % 5 == 0:
                    #print(feedback)

            # Do something depending on the return code
            result = navigator.getResult()
            print('Result: ', result)
            if result == TaskResult.SUCCEEDED:
                #calibrate pose
                print("toto")
                if go_to_omni == True:
                    omni_man.calibrate_pose_pipeline()    
                else:
                    nmr_man.calibrate_pose_pipeline()
                go_to_omni != go_to_omni
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled! Waiting for new orders!')
            elif result == TaskResult.FAILED:
                print('Goal failed, returning home!')
            else:
                print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()