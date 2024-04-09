from geometry_msgs.msg import PoseStamped
from enum import Enum

class Station:
    def __init__(self, posX, posY, orientationZ, orientationW):
        """
        Initializes a Station object with a given position and orientation.

        Args:
        - posX: float, the x-coordinate of the station's position
        - posY: float, the y-coordinate of the station's position
        - orientationZ: float, the z-coordinate of the station's orientation
        - orientationW: float, the w-coordinate of the station's orientation
        """
        self.position = {
            "x": posX,
            "y": posY,
            "z": 0.0
        }
        self.orientation = {
            "x": 0.0,
            "y": 0.0,
            "z": orientationZ,
            "w": orientationW
        }
        self.is_occupied = False # True if a robot is currently on the station
        
        self.station_name = {
            'REST': [-0.205, 7.403],
            'NMR': [-0.073, -8.497],
            'OMNI': [6.217, 2.153],
        }
        
    def define_goal_pose(self):
        """
        Creates a PoseStamped object representing the goal pose of the station.

        Returns:
        - goal_pose: PoseStamped object, the goal pose of the station
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = self.position["x"]
        goal_pose.pose.position.y = self.position["y"]
        goal_pose.pose.position.z = self.position["z"]
        goal_pose.pose.orientation.x = self.orientation["x"]
        goal_pose.pose.orientation.y = self.orientation["y"]
        goal_pose.pose.orientation.z = self.orientation["z"]
        goal_pose.pose.orientation.w = self.orientation["w"]
        return goal_pose
    
class Robot:
    class Type(Enum):
        TRANSPORTER = 0
        RESCUER = 1 
        CLEANER = 2
    # State = {
    #     'standby': 0,
    #     'moving': 1,
    #     'on_station': 2

    # }
    class State(Enum):
        STANDBY = 0
        MOVING = 1
        ON_STATION = 2
        RETURNING_HOME = 3
        ERROR = 4

    def __init__(self, robot_type=Type.TRANSPORTER, state=State.STANDBY, goals=[], empty=True, current_pose=PoseStamped()):
        self.robot_type = robot_type
        self.state = state
        self.goal_poses = goals
        self.is_empty = empty #  False if the robot carries a load. No meaning for the cleaner type
        self.sample_ID = 0 # corresponds to the ID number of the actual sample being carried, specific to the transporter type, set to 0 for other types
        # self.current_pose = current_pose not sure if needed 
        # self.current_goal = None # not sure if needed
        # self.battery_level = 90.0 # not sure if needed                                                                                                                                                                                                    
        # self.lights = False # if we put lights, can be specified here whether we want to enable them or not, again not sure if needed

    # def is_charged(self):
    #     """
    #     Checks if the robot is charged.

    #     Returns:
    #     - is_charged: bool, True if the robot is charged, False otherwise
    #     """
    #     return self.battery_level >= 95.0

    # def check_load(self):
    #     """
    #     Checks if there was a problem with the load.

    #     Returns:
    #     - is_empty: bool, True if the robot is empty, False otherwise
    #     """
    #     if Robot.type.TRANSPORTER:
    #         if (not self.is_empty) and 
    #     elif Robot.type.RESCUER:
    #         return True
    #     elif Robot.type.CLEANER:
    #         return True
