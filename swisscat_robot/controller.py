#!/usr/bin/env python3

# test

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
import numpy as np
from rclpy.qos import QoSProfile
import time
from geometry_msgs.msg import PoseStamped

VERBOSE = False
DIST_THRESH = 0.3


class DiffDriveRobot(Node):

    def __init__(self, robot_name='edymobile', init_pos=[-3.18, -3.595], init_heading=0.0, init_vel_cmd=[0.0, 0.0],
                 v_cst=0.1):
        super().__init__('controller')
        self.robot_name = robot_name
        # Pose
        self.x = init_pos[0]
        self.y = init_pos[1]
        self.theta = init_heading
        # Velocity command
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = init_vel_cmd[0]
        self.cmd_vel.angular.z = init_vel_cmd[1]
        self.v_desired = v_cst
        # Position goal
        self.goal = PoseStamped()
        self.target_x = None
        self.target_y = None
        self.goal_reached = False
        self.dist_to_goal = None
        # Control gains
        self.Ka = 0.1
        self.Kv = 0.2
        self.ki = 0.00008
        self.kd = 0.001
        self.tot_error = 0.0

        # Initialize ROS2 communications
        qos_profile = QoSProfile(depth=10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.poseCallBack, qos_profile)
        self.goal_sub = self.create_subscription(PoseStamped, 'navigate_to_pose/goal', self.goalCallBack, qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        # TF2 Buffer and Listener
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

    def set_target(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
        self.goal_reached = False

    def set_cmd_vel(self, v, w):
        self.cmd_vel.linear.x = float(v)
        self.cmd_vel.angular.z = float(w)

    def compute_vel(self):
        # Calculate the linear error
        self.dist_to_goal = math.sqrt((self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2)

        if VERBOSE:
            self.get_logger().info(self.robot_name + ': Current x: ' + str(self.x))
            self.get_logger().info(self.robot_name + ': New x target: ' + str(self.target_x))
            self.get_logger().info(self.robot_name + ': Current y: ' + str(self.y))
            self.get_logger().info(self.robot_name + ': New y target: ' + str(self.target_y))

        if abs(self.target_x - self.x) > DIST_THRESH or abs(self.target_y - self.y) > DIST_THRESH or min(
                abs(self.target_y - self.y), abs(self.target_x - self.x)) > 0.1:
            heading = math.atan2(self.target_y - self.y, self.target_x - self.x)
            alpha = heading - self.theta

            if VERBOSE:
                self.get_logger().info(self.robot_name + ': heading to goal ' + str(heading))
                self.get_logger().info(self.robot_name + ': theta robot in map ' + str(self.theta))

            if abs(alpha) > math.pi:
                alpha -= 2 * math.pi
            elif alpha < -math.pi:
                alpha += 2 * math.pi

            if VERBOSE:
                self.get_logger().info(self.robot_name + ': angular error: ' + str(alpha))

            self.tot_error += alpha

            if VERBOSE:
                self.get_logger().info(self.robot_name + ': heading to goal ' + str(heading))
                self.get_logger().info(self.robot_name + ': theta robot in map ' + str(self.theta))
                self.get_logger().info(self.robot_name + ': angular error: ' + str(alpha))

            if abs(alpha) > 0.5 * math.pi:
                v_des = -self.v_desired
                if VERBOSE:
                    self.get_logger().info(self.robot_name + ' Should move backwards')
            else:
                v_des = self.v_desired

            v_error = v_des - self.cmd_vel.linear.x
            change_dir = (abs(alpha) > 0.05 * math.pi and abs(alpha) < 0.8 * math.pi)

            if VERBOSE:
                self.get_logger().info(self.robot_name + ': change dir ? ' + str(change_dir))

            self.cmd_vel.linear.x = (v_des + self.Kv * v_error) * (not change_dir)
            self.cmd_vel.angular.z = -self.Ka * (alpha + 4 * np.sign(alpha) * change_dir) * np.sign(v_des)

        else:
            if VERBOSE:
                self.get_logger().info(self.robot_name + ': Goal reached')
            self.goal_reached = True
            self.set_cmd_vel(0, 0)
            self.tot_error = 0

    def poseCallBack(self, odom_msg):
        '''
        Get pose information from robot odometry topic
        '''
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        (roll, pitch, self.theta) = euler_from_quaternion(
            [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z,
             odom_msg.pose.pose.orientation.w])

    def goalCallBack(self, navigateToPoseGoal_msg):
        '''
        Get next goal sent by fleet manager
        '''
        self.goal = navigateToPoseGoal_msg
        self.target_x = self.goal.pose.position.x
        self.target_y = self.goal.pose.position.y


def main(args=None):
    rclpy.init(args=args)

    controller = DiffDriveRobot()

    loop_rate = 0.1  # Corresponds to 10Hz

    while rclpy.ok():
        start_time = time.time()

        if controller.target_x is None or controller.target_y is None:
            controller.set_cmd_vel(0, 0)
        else:
            controller.compute_vel()
            if VERBOSE:
                controller.get_logger().error("Target X : " + controller.target_x + "Target Y : " + controller.target_y)
                controller.get_logger().info(controller.robot_name + ' Forward Velocity command: ' +
                                             str(controller.cmd_vel.linear.x) + ' angular: ' +
                                             str(controller.cmd_vel.angular.z))
        controller.cmd_vel_pub.publish(controller.cmd_vel)
        rclpy.spin_once(controller)

        elapsed_time = time.time() - start_time
        if elapsed_time < loop_rate:
            time.sleep(loop_rate - elapsed_time)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
