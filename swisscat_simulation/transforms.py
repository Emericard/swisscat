import rclpy
from rclpy.node import Node, QoSProfile
import tf2_ros
import math
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry

# https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html

class TransformPublisher(Node):
    def __init__(self):
        super().__init__("transform_publisher_node")

        # Declare and acquire `turtlename` parameter
        # self.robotname = self.declare_parameter(
        #   'robotname', 'robot').get_parameter_value().string_value
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribe to your odometry topic
        # self.sub_odom = self.create_subscription(
        #     Odometry,
        #     f'/{self.robotname}/odometry/global',
        #     self.odom_callback,
        #     10
        # )
        self.sub_odom = self.create_subscription(
            Odometry,
            'odometry/global',
            self.odom_callback,
            10
        )

        # Initialize transformation variables
        self.odom_translation_x = 0.0
        self.odom_translation_y = 0.0
        self.odom_translation_z = 0.0
        self.odom_rotation_qx = 0.0
        self.odom_rotation_qy = 0.0
        self.odom_rotation_qz = 0.0
        self.odom_rotation_qw = 1.0

    def odom_callback(self, msg):
        # Update the transformation variables from the received odometry message
        self.odom_translation_x = msg.pose.pose.position.x
        self.odom_translation_y = msg.pose.pose.position.y
        self.odom_translation_z = msg.pose.pose.position.z
        self.odom_rotation_qx = msg.pose.pose.orientation.x
        self.odom_rotation_qy = msg.pose.pose.orientation.y
        self.odom_rotation_qz = msg.pose.pose.orientation.z
        self.odom_rotation_qw = msg.pose.pose.orientation.w

        # Publish transformations
        self.publish_transforms()

    def publish_transforms(self):
        # Create transformation messages
        map_to_odom = self.map_transform()
        odom_to_base_link = self.odom_transform()
        # baselink_to_marv = self.baselink_transform()
        # world_to_map = self.create_transform_world("world", "map")

        # Broadcast transformations
        #self.tf_broadcaster.sendTransform(world_to_map)
        self.tf_broadcaster.sendTransform(map_to_odom)
        self.tf_broadcaster.sendTransform(odom_to_base_link)
        # self.tf_broadcaster.sendTransform(baselink_to_marv)

    def odom_transform(self):
        
        # Create a TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        # Set the translation and rotation components of the transform
        transform.transform.translation.x = self.odom_translation_x
        transform.transform.translation.y = self.odom_translation_y
        transform.transform.translation.z = self.odom_translation_z
        transform.transform.rotation.x = self.odom_rotation_qx
        transform.transform.rotation.y = self.odom_rotation_qy
        transform.transform.rotation.z = self.odom_rotation_qz
        transform.transform.rotation.w = self.odom_rotation_qw

        return transform
    
    def map_transform(self):
        # Create a TransformStamped message
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'map' 
        transform_stamped.child_frame_id = 'odom' 
        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 0.0
        transform_stamped.transform.translation.z = 0.0

        # Rotate map frame by 180 degrees around the Z-axis (yaw)
        roll = math.pi
        yaw = 1.54 #1.5008 # 90 degrees in radians
        quaternion = quaternion_from_euler(roll, 0.0, 0.0)
        transform_stamped.transform.rotation.w = quaternion[0]
        transform_stamped.transform.rotation.x = quaternion[1]
        transform_stamped.transform.rotation.y = quaternion[2]
        transform_stamped.transform.rotation.z = quaternion[3]

        return transform_stamped
    
    # def baselink_transform(self):
    #     # Create a TransformStamped message
    #     transform_stamped = TransformStamped()
    #     transform_stamped.header.stamp = self.get_clock().now().to_msg()
    #     transform_stamped.header.frame_id = 'base_link' 
    #     transform_stamped.child_frame_id = 'marvelmind' 
    #     transform_stamped.transform.translation.x = -0.091
    #     transform_stamped.transform.translation.y = 0.0
    #     transform_stamped.transform.translation.z = 0.073

    #     # Rotate map frame by 180 degrees around the Z-axis (yaw)
    #     roll = math.pi
    #     yaw = 1.54 #1.5008 # 90 degrees in radians
    #     quaternion = quaternion_from_euler(roll, 0.0, 0.0)
    #     transform_stamped.transform.rotation.w = quaternion[0]
    #     transform_stamped.transform.rotation.x = quaternion[1]
    #     transform_stamped.transform.rotation.y = quaternion[2]
    #     transform_stamped.transform.rotation.z = quaternion[3]

    #     return transform_stamped

def main(args=None):
    rclpy.init(args=args)
    transform_publisher = TransformPublisher()
    rclpy.spin(transform_publisher)
    transform_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
