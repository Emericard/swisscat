import rclpy
from tf_transformations import quaternion_from_euler
from rclpy.node import Node
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

import tf2_ros
import math

# Constants & Global Variables
initialX = 0.0
initialY = 0.0
initialTheta = 0.00000000001
PI = 3.141592
TICKS_PER_REVOLUTION = 1795.92 / 2
TICKS_PER_REV_INVERSE = 0.00111364
WHEEL_RADIUS = 0.034
WHEEL_BASE = 0.2085
WHEEL_BASE_INVERSE = 4.7961
TICKS_PER_METER = 4083.27921997
TICKS_PER_METER_INV = 0.000244140625

distanceLeft = 0
distanceRight = 0
cmpt = 0
lastCountL = 0
lastCountR = 0
save_rightTicks = 0
save_leftTicks = 0

class OdomPublisher(Node):

    def __init__(self):
        super().__init__('ticks2odom_node')
        # Initialization of odomNew & odomOld
        self.get_logger().info("Node odom_pub initialized")
        self.odomNew = Odometry()
        self.odomOld = Odometry()
        self.odomNew.header.frame_id = "odom"
        self.odomNew.pose.pose.position.z = 0.0
        self.odomNew.pose.pose.orientation.x = 0.0
        self.odomNew.pose.pose.orientation.y = 0.0
        self.odomNew.twist.twist.linear.x = 0.0
        self.odomNew.twist.twist.linear.y = 0.0
        self.odomNew.twist.twist.linear.z = 0.0
        self.odomNew.twist.twist.angular.x = 0.0
        self.odomNew.twist.twist.angular.y = 0.0
        self.odomNew.twist.twist.angular.z = 0.0
        self.odomNew.pose.covariance = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.03]
        self.odomNew.twist.covariance = [0.08, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.08, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.03]
        self.odomOld.pose.covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
        self.odomOld.twist.covariance = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
        self.odomOld.pose.pose.position.x = initialX
        self.odomOld.pose.pose.position.y = initialY
        self.odomOld.pose.pose.orientation.z = initialTheta
        self.timeOld_sec = self.odomNew.header.stamp.sec
        self.timeOld_nanosec = self.odomNew.header.stamp.nanosec
        self.tf_buffer = tf2_ros.Buffer()

        current_logger_level = self.get_logger().get_effective_level()
        self.get_logger().info(f"Logger level is set to: {current_logger_level}")

        # Publishers and Subscribers
        self.subForRightCounts = self.create_subscription(Int16, 'right_ticks_counts', self.calc_right, 10)
        self.subForLeftCounts = self.create_subscription(Int16, 'left_ticks_counts', self.calc_left, 10)
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)

        timer_frequency = 10.0  
        self.timer = self.create_timer(1.0 / timer_frequency, self.publish_data)

    def calc_left(self, msg):
        global distanceLeft, lastCountL
        if msg.data != 0 and lastCountL != 0:
            leftTicks = msg.data - lastCountL
            if leftTicks > 10000:
                self.get_logger().info(f"LEFT TICKS: {leftTicks}, LAST_COUNT: {lastCountL}, MSG_DATA: {msg.data}")
                leftTicks = save_leftTicks #0 - (32767 - leftTicks)
            elif leftTicks < -10000:
                leftTicks = save_leftTicks #32767 - leftTicks
                self.get_logger().info(f"LEFT TICKS: {leftTicks}, LAST_COUNT: {lastCountL}, MSG_DATA: {msg.data}")
            distanceLeft = leftTicks * TICKS_PER_METER_INV
            save_leftTicks = leftTicks
        lastCountL = msg.data

    def calc_right(self, msg):
        global distanceRight, lastCountR
        if msg.data != 0 and lastCountR != 0:
            rightTicks = msg.data - lastCountR
            if rightTicks > 10000:
                self.get_logger().info(f"RIGHT TICKS: {rightTicks}, LAST_COUNT: {lastCountR}, MSG_DATA: {msg.data}")
                rightTicks = save_rightTicks #0 - (32767 - rightTicks)
            elif rightTicks < -10000:
                rightTicks = save_rightTicks #32767 - rightTicks
            distanceRight = rightTicks * TICKS_PER_METER_INV
            save_rightTicks = rightTicks
        lastCountR = msg.data

    def publish_odom(self):
        q = Quaternion()
        q = quaternion_from_euler(0, 0, self.odomNew.pose.pose.orientation.z)

        odom_out = Odometry()
        odom_out.header.stamp = self.odomNew.header.stamp
        odom_out.header.frame_id = "odom"
        odom_out.child_frame_id = "base_link"
        odom_out.pose.pose.position.x = self.odomNew.pose.pose.position.x
        odom_out.pose.pose.position.y = self.odomNew.pose.pose.position.y
        odom_out.pose.pose.position.z = self.odomNew.pose.pose.position.z
        odom_out.pose.pose.orientation.x = q[0]
        odom_out.pose.pose.orientation.y = q[1]
        odom_out.pose.pose.orientation.z = q[2]
        odom_out.pose.pose.orientation.w = q[3]
        odom_out.twist.twist = self.odomNew.twist.twist
        self.get_logger().debug(f"ODOM_OUT X POSE:{odom_out.pose.pose.position.x}")     
    
        for i in range(36):
            if i in [0, 7, 14]:
                odom_out.pose.covariance[i] = 0.3
            elif i in [21, 28, 35]:
                odom_out.pose.covariance[i] += 0.0
            else:
                odom_out.pose.covariance[i] = 0.0

        self.odom_pub.publish(odom_out)

    def update_odom(self):
        global distanceLeft, distanceRight, cmpt
        cycleDistance = (distanceRight + distanceLeft) / 2
        arg_sin = (distanceRight - distanceLeft) * WHEEL_BASE_INVERSE
        if cycleDistance > 0.04:
            cycleDistance = 0.04
        if arg_sin > 1:
            arg_sin = -0.98
        elif arg_sin < -1:
            arg_sin = 0.98
        try:
            cycleAngle = math.asin(arg_sin)
            cmpt +=1
            if cmpt % 10 == 0:
                self.get_logger().debug(f"CYCLE_DISTANCE: {cycleDistance}, ARG_SIN: {arg_sin}, CYCLE_ANGLE: {cycleAngle}")
        except:
            self.get_logger().error('math sin exception, cycle angle set to 0')
            self.get_logger().error(f"CYCLE_DISTANCE: {cycleDistance}, ARG_SIN: {arg_sin}")
            cycleAngle = 0.0
            
        avgAngle = cycleAngle*0.5 + self.odomOld.pose.pose.orientation.z

        if avgAngle > PI:
            avgAngle -= 2 * PI
        elif avgAngle < -PI:
            avgAngle += 2 * PI

        self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x + math.cos(avgAngle) * cycleDistance
        self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y + math.sin(avgAngle) * cycleDistance
        self.odomNew.pose.pose.orientation.z = cycleAngle + self.odomOld.pose.pose.orientation.z

        if math.isnan(self.odomNew.pose.pose.position.x) or math.isnan(self.odomNew.pose.pose.position.y): #or math.isnan(self.odomNew.pose.pose.position.z):
            self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x
            self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y
            self.odomNew.pose.pose.orientation.z = self.odomOld.pose.pose.orientation.z

        if self.odomNew.pose.pose.orientation.z > PI:
            self.odomNew.pose.pose.orientation.z -= 2 * PI
        elif self.odomNew.pose.pose.orientation.z < -PI:
            self.odomNew.pose.pose.orientation.z += 2 * PI

        self.odomNew.header.stamp = self.get_clock().now().to_msg()

        delta_time = (self.odomNew.header.stamp.sec + self.odomNew.header.stamp.nanosec * 1e-9) - ( 
                    self.timeOld_sec + self.timeOld_nanosec * 1e-9)

        if (delta_time == 0):
            self.odomNew.twist.twist.linear.x = self.odomOld.twist.twist.linear.x
            self.odomNew.twist.twist.linear.x = self.odomOld.twist.twist.linear.x
        else:
            self.odomNew.twist.twist.linear.x = cycleDistance / delta_time
            self.odomNew.twist.twist.angular.z = cycleAngle / delta_time

        cycleAngle = 0.0
        self.odomOld = self.odomNew

        self.timeOld_sec = self.odomNew.header.stamp.sec
        self.timeOld_nanosec = self.odomNew.header.stamp.nanosec

    def publish_data(self):
        self.update_odom()
        self.publish_odom()


def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)

    # Destroy the node explicitly
    odom_publisher.destroy_node()
    rclpy.shutdown()
    tf_bugger = tf2_ros.buffer()
    tf_bugger

if __name__ == '__main__':
    main()
