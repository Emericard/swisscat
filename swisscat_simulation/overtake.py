import rclpy
from numpy import *
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist
import math
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry

class OvertakeNode(Node):
    def __init__(self):
        super().__init__('overtake_node')
        self.robot1_pos_sub = self.create_subscription(Odometry, '/Robot1/odom', self.robot1_pos_callback, 10)
        self.robot2_pos_sub = self.create_subscription(Odometry, '/Robot2/odom', self.robot2_pos_callback, 10)
        self.pose_pub = self.create_publisher(Twist, '/Robot2/cmd_vel', 10)
        self.occupancy_grid_sub = self.create_subscription(OccupancyGrid, '/map', self.occupancy_grid_callback, 10)
        self.resolution = 0.1  # Example resolution of the occupancy grid (meters per cell)
        self.occupancy_grid_data = None
    
    pub_ = None
    regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
    }
    state_ = 0
    state_dict_ = {
        0: 'find the wall',
        1: 'turn left',
        2: 'follow the wall',
    }

    def robot1_pos_callback(self, msg):
        # Callback function for robot 1 position
        self.robot1_position = msg.pose.pose()
            
    def robot2_pos_callback(self, msg):
        # Callback function for robot 2 position
        self.robot2_position = msg.pose.pose()

    def occupancy_grid_callback(self, msg):
        self.occupancy_grid_data = msg

    def change_state(state):
        global state_, state_dict_
        if state is not state_:
            print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
            state_ = state
    
    def scan(self):
        # Convert robot's position to grid coordinates
        robot_grid_x = int((self.robot_position.linear.x - self.occupancy_grid_data.info.origin.position.x) / self.resolution)
        robot_grid_y = int((self.robot_position.linear.y - self.occupancy_grid_data.info.origin.position.y) / self.resolution)

        # Ray casting to find nearest obstacle
        scan = []
        robot_angle = self.robot1_position.orientation.z()
        for angle in range(0, 360, 5):  # Cast rays in 5 degree increments
            angle_rad = robot_angle + math.radians(angle)
            x_step = math.cos(angle_rad)
            y_step = math.sin(angle_rad)
            x, y = robot_grid_x, robot_grid_y
            distance = 0
            while 0 <= x < self.occupancy_grid_data.info.width and 0 <= y < self.occupancy_grid_data.info.height:
                if self.occupancy_grid_data.data[y * self.occupancy_grid_data.info.width + x] > 0:
                    break  # Obstacle encountered
                distance += 1
                x += x_step
                y += y_step
            scan[angle] = distance
        return scan
    
    def regions_action(self, msg):
        regions = {
            'right':  min(min(msg.ranges[9:26]), 10),
            'fright': min(min(msg.ranges[0:17]), 10),
            'front':  min(min(msg.ranges[-9:9]), 10),
            'fleft':  min(min(msg.ranges[-17:0]), 10),
            'left':   min(min(msg.ranges[-26:-9]), 10),
        }
        return regions
                
            
    def take_action(self):
        regions = self.regions_actions(self.scan())
             
        d = 1.5
        
        if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            self.change_state(2)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
            self.change_state(0)
        else:
            rclpy.loginfo(regions)
    
    def publish_pose(self) :  
        rate = rclpy.Rate(20)
        msg = Twist()
        while not rclpy.is_shutdown():
            if state_ == 0: # find wall
                msg.linear.x = 0.2
                msg.angular.z = -0.3
            elif state_ == 1: # turn left
                msg.angular.z = 0.3
            elif state_ == 2: # follow wall
                msg.linear.x = 0.5
            else:
                rclpy.logerr('Unknown state!')
                    
        rate.sleep()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Robot2/Twist')


def main(args=None):
    rclpy.init(args=args)
    overtake_node = OvertakeNode()
    rclpy.spin(overtake_node)
    overtake_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()