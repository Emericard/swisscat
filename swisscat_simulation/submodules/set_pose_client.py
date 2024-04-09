from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion
from robot_localization.srv import SetPose
from .generic_functions import setup_logger
from rclpy.node import Node
import rclpy
import logging

RAD2DEG = 57.29577951308232

#specify your config file path and log level here:
config_path = 'config/robot_params.ini'
log_level = 'INFO'
logger = setup_logger(filename = 'calib_client.log',logger_name='GOAL_MANAGER',log_level=log_level)

class PositionCalibrationNode(Node):
    def __init__(self, pose_to_set):
        # Set the name of the root logger
        super().__init__('set_pose_client')
        self.pose_to_set = pose_to_set
        self.cli = self.create_client(SetPose, 'set_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetPose.Request()

    def send_request(self):
        self.req.pose = self.pose_to_set
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        return response

# def main(args=None):
#     rclpy.init(args=args)

#     minimal_client = PositionCalibrationNode()
#     rclpy.spin(minimal_client)
#     # minimal_client.send_request()
#     minimal_client.get_logger().info('New pose set')


#     minimal_client.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()