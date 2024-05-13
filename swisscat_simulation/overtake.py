import rclpy
from numpy import *
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist
import math
from nav_msgs.msg import OccupancyGrid

class OvertakeNode(Node):
    def __init__(self):
        super().__init__('overtake_node')
        self.robot1_pos_sub = self.create_subscription(Twist, '/Robot1/odom', self.robot1_pos_callback, 10)
        self.robot2_pos_sub = self.create_subscription(Twist, '/Robot2/odom', self.robot2_pos_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/Robot2/set_pose', 10)
        self.occupancy_grid_sub = self.create_subscription(OccupancyGrid, '/map', self.occupancy_grid_callback, 10)
        self.resolution = 0.1  # Example resolution of the occupancy grid (meters per cell)
        self.occupancy_grid_data = None
    
    def robot1_pos_callback(self, msg):
        # Callback function for robot 1 position
        self.robot1_position = msg
            
    def robot2_pos_callback(self, msg):
        # Callback function for robot 2 position
        self.robot2_position = msg

    def occupancy_grid_callback(self, msg):
        self.occupancy_grid_data = msg
    
    def calculate_control_command(self):
        # Assuming position is of type geometry_msgs/Point
        robot1_x = self.robot1_position.linear.x
        robot1_y = self.robot1_position.linear.y
        robot2_x = self.robot2_position.linear.x
        robot2_y = self.robot2_position.linear.y
        distance = sqrt((robot1_x-robot2_x)**2+(robot1_y-robot2_y)**2)
        # Check if robots are close
        if distance < 500: # Adjust value for robot size
                # Assuming robot1 is at rest
                # You might need to subscribe to robot velocities for a more accurate calculation
                robot1_speed = 1.0  # Speed of robot1 in units per second

                # Calculate time required for robot1 to reach robot2's position
                time_to_overtake = distance / robot1_speed
                # You might need additional logic if they have different speeds
                overtaking_margin = 1.0  # Margin to start overtaking in seconds

                if time_to_overtake < overtaking_margin:
                    # Generate a control command for robot1 to overtake
                    self.publish_pose()
            
        # If conditions are not met, no overtaking needed
        return self.publish_pose_test()

    def publish_pose_test(self):
        robot1_x = self.robot1_position.linear.x
        robot1_y = self.robot1_position.linear.y
        if self.occupancy_grid_data is not None:
            angle_to_wall = self.calculate_distance_to_nearest_wall()[1]
            pos_x = robot1_x + math.cos(angle_to_wall)
            pos_y = robot1_y + math.sin(angle_to_wall)
        pose_msg = PoseWithCovarianceStamped()
        # Set the header information (frame ID and timestamp)
        pose_msg.header.frame_id = 'map'  # Frame ID of the pose
        pose_msg.header.stamp = self.get_clock().now().to_msg()  # Current time
        # Set the position and orientation of the pose
        pose_msg.pose.position.x = pos_x  # Example x position
        pose_msg.pose.position.y = pos_y  # Example y position

        self.publisher_.publish(pose_msg)
        self.get_logger().info('Publishing pose')

    def publish_pose(self):
        robot1_x = self.robot1_position.linear.x
        robot1_y = self.robot1_position.linear.y
        if self.occupancy_grid_data is not None:
            angle_to_wall = self.calculate_distance_to_nearest_wall()[1]
            pos_x = robot1_x + math.cos(angle_to_wall)
            pos_y = robot1_y + math.sin(angle_to_wall)
        pose_msg = PoseWithCovarianceStamped()
        # Set the header information (frame ID and timestamp)
        pose_msg.header.frame_id = 'map'  # Frame ID of the pose
        pose_msg.header.stamp = self.get_clock().now().to_msg()  # Current time
        # Set the position and orientation of the pose
        pose_msg.pose.position.x = pos_x  # Example x position
        pose_msg.pose.position.y = pos_y  # Example y position

        self.publisher_.publish(pose_msg)
        self.get_logger().info('Publishing pose')


    def calculate_distance_to_nearest_wall(self):
        # Convert robot's position to grid coordinates
        robot_grid_x = int((self.robot_position.linear.x - self.occupancy_grid_data.info.origin.position.x) / self.resolution)
        robot_grid_y = int((self.robot_position.linear.y - self.occupancy_grid_data.info.origin.position.y) / self.resolution)

        # Ray casting to find nearest obstacle
        min_distance = float('inf')
        for angle in range(0, 360, 5):  # Cast rays in 5 degree increments
            angle_rad = math.radians(angle)
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
            min_distance_final = min(min_distance, distance)
            if min_distance_final == distance :
                min_distance_angle = angle_rad

        return [min_distance * self.resolution, min_distance_angle]

def main(args=None):
    rclpy.init(args=args)
    overtake_node = OvertakeNode()
    rclpy.spin(overtake_node)
    overtake_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()