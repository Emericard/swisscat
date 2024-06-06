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
        self.pose_pub_1 = self.create_publisher(Twist, '/Robot1/cmd_vel', 10)
        self.resolution = 0.1  # Example resolution of the occupancy grid (meters per cell)
        self.state_ = 0
        self.speed = 0
        self.robot1_position = Odometry()

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
        nb_rays = 72
        # Ray casting to find nearest obstacle
        scan = [None]*nb_rays
        robot_angle = self.robot1_position.pose.pose.orientation.z
        for i in range(nb_rays):  # Cast rays in 5 degree increments
            range_angles = range(0,360,int(360/nb_rays))
            angle_rad = robot_angle + math.radians(range_angles[i])
            if abs(self.angle_difference_radians(angle_rad, 0)) > math.pi/2 :
                distance = sqrt(((self.robot2_position.pose.pose.position.x-0.15))**2+(((self.robot2_position.pose.pose.position.x-0.15)*math.tan(angle_rad))**2))
            else :
                distance = sqrt(((self.robot2_position.pose.pose.position.x-0.75))**2+(((self.robot2_position.pose.pose.position.x-0.75)*math.tan(angle_rad))**2))
            scan[i] = distance
        return scan
    
    def regions_action(self, msg):
        regions = {
            'right':  min(msg[9:26]),
            'fright': min(msg[0:17]),
            'front':  min(min(msg[0:17], msg[-18:-1])),
            'fleft':  min(msg[54:72]),
            'left':   min(msg[45:63]),
        }
        return regions
                
    def take_action(self):
        distance_to_robot = sqrt(((self.robot2_position.pose.pose.position.x-self.robot1_position.pose.pose.position.x))**2+(self.robot2_position.pose.pose.position.y-self.robot1_position.pose.pose.position.y)**2)
        if distance_to_robot > 0.2 :
            # set speed 
            if distance_to_robot < 2 and distance_to_robot> 0.2 :
                self.speed = distance_to_robot * 0.01
                midline = 0.25
                margin = 0.2
            else : 
                self.speed = 0.02
                midline = 0.45
                margin = 0.15
            if self.robot2_position.pose.pose.position.x > midline : 
                if self.robot2_position.pose.pose.position.x > midline + margin:
                    self.change_state(1)
                else :
                    self.change_state(2)
            else :
                if self.robot2_position.pose.pose.position.x < midline - margin:
                    self.change_state(3)
                else :
                    self.change_state(4)
        else :
            self.change_state(99)
        self.publish_pose()
        self.get_logger().info(str(self.speed))



    def take_action_scan(self):
        regions = self.regions_action(self.scan())
        d = 0.1

        distance_to_robot = sqrt(((self.robot2_position.pose.pose.position.x-self.robot1_position.pose.pose.position.x))**2+(self.robot2_position.pose.pose.position.y-self.robot1_position.pose.pose.position.y)**2)
        if distance_to_robot > 1.5:
            if regions['front'] > 3*d and regions['left'] > d and regions['right'] > d:
                self.change_state(0)
            elif regions['front'] > 3*d and regions['left'] > d and regions['right'] < d:
                self.change_state(4)
            elif regions['front'] > 3*d and regions['left'] < d and regions['right'] > d:
                self.change_state(2)
            elif regions['front'] > 3*d and regions['left'] < d and regions['right'] < d:
                self.change_state(0)
            elif regions['front'] < 3*d and regions['left'] > d and regions['right'] > d:
                self.change_state(0)
            elif regions['front'] < 3*d and regions['left'] > d and regions['right'] < d:
                self.change_state(3)
            elif regions['front'] < 3*d and regions['left'] < d and regions['right'] > d:
                self.change_state(1)
            elif regions['front'] < 3*d and regions['left'] < d and regions['right'] < d:
                self.change_state(1)
        else :
            self.change_state(99)
        self.publish_pose()
        self.get_logger().info(str(self.state_))

    def publish_pose(self) :  
        msg = Twist()
        if self.state_ == 0: # forward 
            msg.linear.x = self.speed * 5
        elif self.state_ == 1: # turn left
            msg.linear.x = self.speed * 1.0
            msg.angular.z = - self.speed * 3
        elif self.state_ == 3: # turn right
            msg.linear.x = self.speed * 1.0
            msg.angular.z = self.speed * 3
        elif self.state_ == 2: # go forward and left
            msg.linear.x = self.speed * 5
            msg.angular.z = - self.speed * 2
        elif self.state_ == 4: # go forward and right
            msg.linear.x = self.speed * 5
            msg.angular.z = self.speed * 2
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Overtake distance ')
                    
        self.pose_pub.publish(msg)
        msg_1 = Twist()
        msg_1.linear.x = 0.02
        self.pose_pub_1.publish(msg_1)
        #self.get_logger().info('Publishing Robot2/Twist')

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