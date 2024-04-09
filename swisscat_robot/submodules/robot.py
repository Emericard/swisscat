#!/usr/bin/env python3

import os
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from .point import Point


class Transformation:
    def __init__(self, tx, ty, angle_rad):
        self.tx = tx
        self.ty = ty
        self.angle_rad = angle_rad
        self.matrix = self.generate_transformation_matrix()

    def generate_transformation_matrix(self):
        cos_a = np.cos(self.angle_rad)
        sin_a = np.sin(self.angle_rad)

        return np.array([[cos_a, -sin_a, self.tx],
                         [sin_a, cos_a, self.ty],
                         [0, 0, 1]])

    def transform_point(self, point):
        extended_point = np.array([point[0], point[1], 1])
        transformed = self.matrix @ extended_point
        return (round(transformed[0], 2), round(transformed[1], 2))

    def inverse_transform_point(self, point):
        inverse_matrix = np.linalg.inv(self.matrix)
        extended_point = np.array([point[0], point[1], 1])
        transformed = inverse_matrix @ extended_point
        return (round(transformed[0], 2), round(transformed[1], 2))


def get_script_path():
    return os.path.dirname(os.path.realpath(__file__))


ini_file_path = os.path.join(get_script_path(), 'configrobots.ini')
circuit_file_path = os.path.join(get_script_path(), 'circuit_MAPF.txt')

VERBOSE = False
DIST_THRESH = 0.3


class Robot(Node):
    def __init__(self, name, initX, initY, id):
        super().__init__(name + "_node")
        self.name = name
        self.id = id
        # robot initial position
        self.posGrid = Point(initX, initY)
        # the goals are stored in descending order of priority: the first goal to reach (the collecting station) is the last in the list
        self.goal = [Point(initX, initY)]
        self.path = []  # path in MAPF frame
        #self.produit = np.array([[0, 1, -3.6], [-1, 0, 4.5], [0, 0, 1]]) @ (self.posGrid.x / 2.1, self.posGrid.y / 2.1, 1)
        self.mapf_to_gazebo = Transformation(-3.6, 4.5, -np.pi / 2)
        self.gazebo_to_mapf = Transformation(3.6, -4.5, np.pi / 2)
        #self.posMap = Point(round(self.produit[0], 2), round(self.produit[1], 2))
        path_temp = (self.posGrid.x / 2.1, self.posGrid.y / 2.1)
        transformed_point = self.mapf_to_gazebo.transform_point(path_temp)
        self.posMap = Point(round(transformed_point[0], 2), round(transformed_point[1], 2))
        self.goalTemp = Point()
        self.path_gazebo = []  # path in Gazebo reference frame
        self.change_dir_idx = []  # store indices in path at which there is a change in direction
        # system state variables
        self.state_checkpoint = 0  # Arrived to the checkpoint? 0: arrived, 1: on the way
        self.state_mission = 0  # All the tasks are achieved? 0: free, 1: still is performing a mission (one mission = a set of tasks)
        self.state_current_task = 0  # Is the current task achieved? 0: finished the task, 1: is performing the task
        self.checkpoint = []

        # ROS Publisher for sending goal to controller
        qos_profile = QoSProfile(depth=10)
        self.pub = self.create_publisher(PoseStamped, '/' + self.name + '/navigate_to_pose/goal', qos_profile)
        self.create_subscription(Odometry, '/' + self.name + '/odom', self.get_odom, qos_profile)

    def get_odom(self, odom_data):
        """
        Get data from odometry topic
        """
        self.posMap.set_coordinates(round(odom_data.pose.pose.position.x, 2), round(odom_data.pose.pose.position.y, 2))

    def base_MAPF2Gazebo(self):
        '''
        Change of base MAPF 2 GAZEBO
        - input : paths in the MAPF frame
        - output: paths in the GAZEBO frame
        '''
        self.path_gazebo.clear()
        for i in range(0, len(self.path)):
            path_temp = (self.path[i][0] / 2.1, self.path[i][1] / 2.1)
            transformed_point = self.mapf_to_gazebo.transform_point(path_temp)
            self.path_gazebo.append(transformed_point)
        #self.path_gazebo.clear()
        #matrix_chg_base = np.array([[0, 1, -3.6], [-1, 0, 4.5], [0, 0, 1]])
        #for i in range(0, len(self.path)):
         #   path_temp = (self.path[i][0] / 2.1, self.path[i][1] / 2.1, 1)
          #  produit = matrix_chg_base @ path_temp
           # self.path_gazebo.append((round(produit[0], 2), round(produit[1], 2)))

    def base_Gazebo2MAPF(self):
        '''
        Change of base GAZEBO 2 MAPF
        - input : paths in the MAPF frame
        - output: paths in the GAZEBO frame
        '''
        path_temp = (self.posMap.x, self.posMap.y)
        transformed_point = self.gazebo_to_mapf.transform_point(path_temp)
        self.posGrid.x = int(round(transformed_point[0] * 2.1, 0))
        self.posGrid.y = int(round(transformed_point[1] * 2.1, 0))
        #matrix_chg_base = np.array([[0, -1, 4.5], [1, 0, 3.6], [0, 0, 1]])
        #path_temp = (self.posMap.x, self.posMap.y, 1)
        #produit = matrix_chg_base @ path_temp
        #self.posGrid.x = int(round(produit[0] * 2.1, 0))
        #self.posGrid.y = int(round(produit[1] * 2.1, 0))
        if self.posGrid.x > 19:
            self.posGrid.x = 19
        if self.posGrid.y > 12:
            self.posGrid.x = 12

    def send_goal(self):
        '''
        Publish robot next goal or checkpoint
        '''
        if VERBOSE:
            self.get_logger().info('----SEND GOAL----')

        # Set up the goal
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()  # Use current time
        goal.header.frame_id = "map"  # Assuming the frame of reference is "map"
        goal.pose.position.x = self.path_gazebo[0][0]
        goal.pose.position.y = self.path_gazebo[0][1]
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0  # Assuming no rotation
        self.state_checkpoint = 1  # the robot is on the way to the checkpoint
        self.goalTemp.set_coordinates(self.path_gazebo[0][0], self.path_gazebo[0][1])

        if VERBOSE:
            self.get_logger().info(
                f'new checkpoint or goal for {self.name} {self.path_gazebo[0][0]} {self.path_gazebo[0][1]}')

        # If the robot has reached its goal, make it free again
        if len(self.path_gazebo) < 1:
            if VERBOSE:
                self.get_logger().info(f'same pos {self.path_gazebo[0][0]} {self.path_gazebo[0][1]}')
            self.state_current_task = 0  # the robot is free again

        self.pub.publish(goal)

        return goal
