from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf_transformations import quaternion_from_euler
import configparser
import logging
import os

package_path_from_src = 'src/mob_rob_loca'

def set_ros_pose_stamped(pose_values):
    """
    Set the ROS PoseStamped type based on the given pose values.

    Args:
        pose_values (list): List of pose values [x, y, yaw].

    Returns:
        pose_stamped: The output pose in PoseStamped type.
    """
    pose_stamped = PoseWithCovarianceStamped()
    pose_stamped.header.frame_id = 'map'
    pose_stamped.pose.position.x = float(pose_values[0])
    pose_stamped.pose.position.y = float(pose_values[1])

    yaw = float(pose_values[2]) # radians
    quat = quaternion_from_euler(0.0, 0.0, yaw)
    pose_stamped.pose.orientation.z = quat[2]
    pose_stamped.pose.orientation.w = quat[3]

    return pose_stamped

def set_ros_pose_cov_stamped(pose_values):
    """
    Set the ROS pose based on the given pose values.

    Args:
        pose_values (list): List of pose values [x, y, yaw].

    Returns:
        pose_cov_stamped: The output pose in PoseWithCovarianceStamped type.
    """
    pose_cov_stamped = PoseWithCovarianceStamped()
    pose_cov_stamped.header.frame_id = 'map'
    pose_cov_stamped.pose.pose.position.x = float(pose_values[0])
    pose_cov_stamped.pose.pose.position.y = float(pose_values[1])

    yaw = float(pose_values[2]) # radians
    quat = quaternion_from_euler(0.0, 0.0, yaw)
    pose_cov_stamped.pose.pose.orientation.z = quat[2]
    pose_cov_stamped.pose.pose.orientation.w = quat[3]

    return pose_cov_stamped


def setup_logger(filename = 'default.log',logger_name='root', log_level='INFO'):

    logging.basicConfig(filename=filename, #data_files/calibration_data.csv
                    filemode='a',
                    format='%(asctime)s,%(msecs)d, %(name)s, %(message)s',
                    datefmt='%H:%M:%S',
                    level=logging.INFO)
    
    # Create a logger with the desired name if it doesn't exist
    logger = logging.getLogger(logger_name)

    if not logger.handlers:  
        logger.setLevel(log_level)

    return logger

def setup_config(config_path = 'config/robot_params.ini'):
    # Create a ConfigParser object
    config = configparser.ConfigParser()

    to_pkg_dir = os.path.join(os.getcwd(), package_path_from_src)
    config_dir = os.path.join(to_pkg_dir ,config_path)

    with open(config_dir) as config_file:
        config.read_file(config_file)

    return config

