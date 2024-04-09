from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from .socket_client import SocketClient
from .set_pose_client import PositionCalibrationNode
from .generic_functions import set_ros_pose_cov_stamped, setup_logger, setup_config
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import rtde_io
import rclpy
import logging
import time

#specify your config file path and log level here:
config_path = 'config/robot_params.ini'
log_level = 'INFO'
config = setup_config(config_path=config_path)
logger = setup_logger(filename = 'data.log',logger_name='GOAL_MANAGER',log_level=log_level)
logging.getLogger().name = 'Calibration'

"""
Class charged of sequencing robot actions
"""

class Goal_Manager:
    def __init__(self, goal_station):
        # logging.getLogger().name = 'Calibration'

        self.goal_station = goal_station
        self.port_in_server = config.get(self.goal_station, 'cam_port')
        self.cam_server = config.get('Server', 'cam_ip')

        self.goal_pose = PoseStamped()
        self.status = None

        self.calibrated = False
        self.ur_light_port = int(config.get(self.goal_station, 'ur_light_port'))

        logger.debug('Setting goal pose')
        self.set_ros_goal_pose() 
        logger.debug("Calibrating position")

    def set_ros_goal_pose(self):
        """
        Creates a PoseStamped object representing the goal pose of the station.
        """
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = float(config.get(self.goal_station, 'pos_x'))
        self.goal_pose.pose.position.y = float(config.get(self.goal_station, 'pos_y'))

        yaw_rad = float(config.get(self.goal_station, 'yaw'))
        quat = quaternion_from_euler(0.0, 0.0, yaw_rad)
        self.goal_pose.pose.orientation.z = quat[2]
        self.goal_pose.pose.orientation.w = quat[3]  

    def calibrate_pose_pipeline(self):
        """
        Calibrates the pose pipeline by performing the following steps:
        1. Turns on the light.
        2. Retrieves the pose from the camera.
        3. Sets the updated pose using a service request.
        4. Turns off the light.
        """
        logger.debug('Light ON Request')
        self.toggle_light(True)

        logger.debug('Get pose from camera')
        ros_pose = self.get_accur_pose()
        logger
        logger.debug('SetPose Service Request')
        self.set_updated_pose(ros_pose)

        logger.debug('Light OFF Request')
        self.toggle_light(False)

    def get_accur_pose(self):
        """
        Get the accurate pose from the camera server.

        Returns:
            ros_pose (ROS Pose): The accurate pose in ROS format.
        """
        cli = SocketClient("Client", self.cam_server, int(self.port_in_server))
        cli.connect()
        cli.send_data("Calibrate")
        # first data received is just an acknowledgement one
        ack_data = cli.receive_data() 
        logger.debug('Ack received: %s', ack_data)
        received_pose = cli.receive_data()
        logger.debug('Pose received: %s', received_pose)
        pose_values = received_pose.strip("()").split(";")
        pose_values = [float(value.replace(',', '.')) for value in pose_values]
        # convert to ros type
        ros_pose = set_ros_pose_cov_stamped(pose_values)
        print(ros_pose)
        return ros_pose

    def set_updated_pose(self, pose_to_set):
        """
        Calls the SetPose service of the module robot_localization to set the new pose of the robot.

        """
        angle = euler_from_quaternion([0, 0, pose_to_set.pose.pose.orientation.z, pose_to_set.pose.pose.orientation.w])[2]

        logging.info('CalibratedPos, x:%f, y:%f, rz:%f', pose_to_set.pose.pose.position.x, pose_to_set.pose.pose.position.y, angle)
        
        set_pose_cli = PositionCalibrationNode(pose_to_set)
        set_pose_cli.send_request()
        self.calibrated = True
        set_pose_cli.destroy_node()

    def toggle_light(self, state):
        """
        Toggles the light using the UR pin.

        Args:
            state (bool): The desired state of the light. True to turn on, False to turn off.
        """
        logger.debug('Toggling light of port %s', self.port_in_server)
        rtde_io_interface = rtde_io.RTDEIOInterface(config.get(self.goal_station, 'ur_ip'))
        rtde_io_interface.setStandardDigitalOut(self.ur_light_port, state)
        rtde_io_interface.disconnect()
        self.ur_light_state = state 

    def __del__(self):
        """
        Destructor of the class.
        """
        logger.debug('Destructor called')

def main():
    rclpy.init()

    station_request = 'NMR' # virtual request

    goal_man = Goal_Manager(station_request)

    goal_man.calibrate_pose_pipeline()

    exit(0)

if __name__ == '__main__':
    main()

