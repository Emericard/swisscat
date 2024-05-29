import os
from ament_index_python import get_package_share_directory
import rclpy
import cv2
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

rob_loca_dir = get_package_share_directory('swisscat_simulation')
map_path = os.path.join(rob_loca_dir, 'maps/overtake_map.png')

def create_occupancy_grid_from_image(image_path, resolution):
    # Read the PNG image using OpenCV
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # Determine map dimensions based on image size and resolution
    height, width = image.shape
    map_width = int(width * resolution)
    map_height = int(height * resolution)

    # Resize the image to match the map dimensions
    resized_image = cv2.resize(image, (map_width, map_height), interpolation=cv2.INTER_NEAREST)

    # Convert image data to OccupancyGrid message format
    occupancy_grid_data = []
    for row in resized_image:
        for pixel_value in row:
            # Convert pixel value to occupancy probability (0-100)
            occupancy_prob = 100 - (pixel_value / 255.0) * 100
            # Convert occupancy probability to map data value (-1 for unknown, 0 for free, 100 for occupied)
            if occupancy_prob == 0:
                occupancy_grid_data.append(0)
            else:
                occupancy_grid_data.append(100)

    # Create an OccupancyGrid message
    occupancy_grid_msg = OccupancyGrid()
    occupancy_grid_msg.info.resolution = resolution
    occupancy_grid_msg.info.width = map_width
    occupancy_grid_msg.info.height = map_height
    occupancy_grid_msg.data = occupancy_grid_data

    return occupancy_grid_msg

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.map = create_occupancy_grid_from_image(map_path, 0.1)
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        self.timer_ = self.create_timer(5.0, self.publish_map)
        self.map_width = self.map.info.width  # Define map width in cells
        self.map_height = self.map.info.height  # Define map height in cells

    def generate_map(self):
        # Example: Generating a simple map with random occupancy values
        # Create an OccupancyGrid message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = 0.1  # Set map resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = -5.0
        map_msg.info.origin.position.y = -5.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = self.map.data
        return map_msg

    def publish_map(self):
        map_msg = self.generate_map()
        self.publisher_.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
