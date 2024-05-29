import rclpy
from numpy import *
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist
import math
from nav_msgs.msg import OccupancyGrid, Odometry

class OvertakeNode(Node):
    def __init__(self):
        super().__init__('overtake_node')
        self.occupancy_grid_sub = self.create_subscription(OccupancyGrid, '/map', self.occupancy_grid_callback, 10)
        self.robot1_pos_sub = self.create_subscription(Odometry, '/Robot1/odom', self.robot1_pos_callback, 10)
        self.robot2_pos_sub = self.create_subscription(Odometry, '/Robot2/odom', self.robot2_pos_callback, 10)
        self.pose_pub = self.create_publisher(Twist, '/Robot2/cmd_vel', 10)
        self.resolution = 0.1  # Example resolution of the occupancy grid (meters per cell)
        self.state_ = 0

    pub_ = None
    regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
    }
    state_dict_ = {
        0: 'find the wall',
        1: 'turn left',
        2: 'follow the wall',
    }

    def robot1_pos_callback(self, msg):
        # Callback function for robot 1 position
        self.robot1_position = msg
            
    def robot2_pos_callback(self, msg):
        # Callback function for robot 2 position
        self.robot2_position = msg
        self.take_action()

    def occupancy_grid_callback(self, msg):
        self.occupancy_grid_data = msg


    def angle_difference_radians(self, angle1, angle2):
        """
        Calculate the difference between two angles in radians.
        
        The result is within the range [-π, π].

        :param angle1: First angle in radians.
        :param angle2: Second angle in radians.
        :return: The difference between the two angles in radians.
        """
        diff = (angle1 - angle2) % (2 * math.pi)
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
        return diff
                                    
    def scan(self):
        # Convert robot's position to grid coordinates
        nb_rays = 72
        # Ray casting to find nearest obstacle
        scan = [None]*nb_rays
        robot_angle = self.robot1_position.pose.pose.orientation.z
        for i in range(nb_rays):  # Cast rays in 5 degree increments
            range_angles = range(0,360,int(360/nb_rays))
            angle_rad = robot_angle + math.radians(range_angles[i])
            if abs(self.angle_difference_radians(angle_rad, math.pi)) > pi/2 :
                distance = sqrt(((self.robot2_position.pose.pose.position.x-0.75))**2+(((self.robot2_position.pose.pose.position.x-0.75)*math.tan(angle_rad))**2))
            else :
                distance = sqrt(((self.robot2_position.pose.pose.position.x-0.15))**2+(((self.robot2_position.pose.pose.position.x-0.15)*math.tan(angle_rad))**2))
            scan[i] = distance
        return scan
    
    def regions_action(self, msg):
        regions = {
            'right':  min(msg[9:26]),
            'fright': min(msg[0:17]),
            'front':  min(msg[0:9]),
            'fleft':  min(msg[54:72]),
            'left':   min(msg[45:63]),
        }
        return regions
                
            
    def take_action(self):
        regions = self.regions_action(self.scan())
             
        d = 0.01
        
        if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            self.change_state(0)
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            self.change_state(2)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            self.change_state(0)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
            self.change_state(1)
        
        else:
            rclpy.loginfo(regions)
        self.publish_pose()
    
    def publish_pose(self) :  
        msg = Twist()
        if self.state_ == 0: # find wall
            msg.linear.x = 0.2
            msg.angular.z = -0.3
        elif self.state_ == 1: # turn left
            msg.angular.z = 0.3
        elif self.state_ == 2: # follow wall
            msg.linear.x = 0.5
        else:
            rclpy.logerr('Unknown state!')
                    
        self.pose_pub.publish(msg)
        self.get_logger().info('Publishing Robot2/Twist')

    def change_state(self, state):
        if state is not self.state_:
            self.state_ = state

def main(args=None):
    rclpy.init(args=args)
    overtake_node = OvertakeNode()
    rclpy.spin(overtake_node)
    overtake_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()