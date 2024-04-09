#specify your config file path and log level here:
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from .submodules.generic_functions import setup_logger
from geometry_msgs.msg import PoseWithCovarianceStamped
from marvelmind_ros2_msgs.msg import HedgePosition
from nav_msgs.msg import Odometry
import logging
import tf2_ros

QOS_DEPTH = 10

config_path = 'config/robot_params.ini'
log_level = 'INFO'
logger = setup_logger(filename = 'ground_truth_odom.log', logger_name='Plot',log_level=log_level)
hedge_cmpt = 0
odom_cmpt = 0

class Plot(Node):
    def __init__(self):
        super().__init__('plot')
        qos_profile = QoSProfile(depth=QOS_DEPTH )

        self.hedgehog_pos_sub = self.create_subscription(HedgePosition, 'hedgehog_pos', self.hedgehog_pos_callback, qos_profile)
        self.odometry_global_sub = self.create_subscription(Odometry, 'odometry/global', self.odometry_global_callback, qos_profile)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def hedgehog_pos_callback(self, data):
        # transforms the ros type of marvelmind to a compatible one with the robot_localization package
        global hedge_cmpt
        if hedge_cmpt < 100:
            logging.info('HedgePos, x:%f, y:%f', data.x_m, data.y_m)
            hedge_cmpt += 1
        elif hedge_cmpt == 100:
            hedge_cmpt += 1
            self.get_logger().info('Finished logging hedgehog data')
    def odometry_global_callback(self, data):
        global odom_cmpt
        # transforms the ros type of marvelmind to a compatible one with the robot_localization package
        if odom_cmpt < 100:
            self.get_logger().debug('Logging odom data')
            logging.info('OdomGlob, x:%f, y:%f, rz:%f', data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.z)
            odom_cmpt += 1
        elif odom_cmpt == 100:
            odom_cmpt += 1
            self.get_logger().info('Finished logging odom data')


def main(args=None):
    rclpy.init(args=args)

    plot = Plot()
    rclpy.spin(plot)

    plot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
