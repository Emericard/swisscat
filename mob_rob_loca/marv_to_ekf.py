import rclpy
from rclpy.node import Node
from marvelmind_ros2_msgs.msg import HedgePosition
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Header
from rclpy.qos import QoSProfile
import numpy as np
# from .submodules.generic_functions import setup_logger, setup_config
import tf2_ros

QOS_DEPTH = 10
ACC_THRESHOLD = 25 #mg/LSB
MG2MS2 = 9.80665 / 1000.0
DIST_MARV_CENTER = 0.091 #m
first = True
yaw = 0.0

class MarvToOdometry(Node):
    def __init__(self):
        super().__init__('marv_to_ekf')

        qos_profile = QoSProfile(depth=QOS_DEPTH )

        self.hedgehog_pos_pub = self.create_publisher(PoseWithCovarianceStamped, 'marv/pos', qos_profile)
        self.imu_pub = self.create_publisher(Imu, 'edi/imu', qos_profile)

        self.ang_vel_x = 0.0
        self.ang_vel_y = 0.0

        self.hedgehog_pos_sub = self.create_subscription(HedgePosition, 'hedgehog_pos', self.hedgehog_pos_callback, qos_profile)
        self.imu_sub = self.create_subscription(Imu, 'bno055/imu', self.imu_callback, qos_profile)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def hedgehog_pos_callback(self, data):
        # transforms the ros type of marvelmind to a compatible one with the robot_localization package
        self.get_logger().debug('Received hedge_pos data')
        global yaw, first
        if first==True and yaw != 0.0:
            yaw_init = yaw
            first=False
            yaw = yaw_init - yaw

        hedge_pos_msg = PoseWithCovarianceStamped()
        hedge_pos_msg.header.stamp = self.get_clock().now().to_msg()
        hedge_pos_msg.header.frame_id = 'odom' 
        hedge_pos_msg.pose.pose.position.x = data.x_m# + DIST_MARV_CENTER* np.cos(yaw)
        hedge_pos_msg.pose.pose.position.y = data.y_m + DIST_MARV_CENTER* np.sin(yaw)
        hedge_pos_msg.pose.pose.position.z = 0.0
        hedge_pos_msg.pose.pose.orientation.w = 1.0
        hedge_pos_msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.013, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.005]
        
        self.hedgehog_pos_pub.publish(hedge_pos_msg)

    def imu_callback(self, data):
        # the yaw angle adjusts the orientation to match the map frame, because the y-axis of the map is not aligned with the magnetic North
        # bad method, works but should think of a better way to do this
        yaw_angle = 2.6
        imu_msg = Imu()

        imu_msg = data
        imu_msg.angular_velocity_covariance = [0.1,0.0,0.0,0.0,0.1,0.0,0.0,0.0,0.1]
        self.get_logger().debug('Resulting yaw angle [rad]: {0}'.format(yaw))

        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    marv_to_ekf = MarvToOdometry()
    # init_pose = InitialPose()
    rclpy.spin(marv_to_ekf)
    # rclpy.spin_one(init_pose)
    marv_to_ekf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

