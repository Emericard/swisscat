import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
import tf2_ros
import math

# Constants & Global Variables
initialX = 0.0
initialY = 0.0
initialTheta = 0.00000000001
PI = 3.141592
TICKS_PER_REVOLUTION = 1795.92 / 2
WHEEL_RADIUS = 0.034
WHEEL_BASE = 0.2085
TICKS_PER_METER = 4083.27921997

distanceLeft = 0
distanceRight = 0

lastCountL = 0
lastCountR = 0


class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_pub')

        # Initialization of odomNew & odomOld
        self.odomNew = Odometry()
        self.odomOld = Odometry()
        self.odomNew.header.frame_id = "odom"
        self.odomNew.pose.pose.position.z = 0
        self.odomNew.pose.pose.orientation.x = 0
        self.odomNew.pose.pose.orientation.y = 0
        self.odomNew.twist.twist.linear.x = 0
        self.odomNew.twist.twist.linear.y = 0
        self.odomNew.twist.twist.linear.z = 0
        self.odomNew.twist.twist.angular.x = 0
        self.odomNew.twist.twist.angular.y = 0
        self.odomNew.twist.twist.angular.z = 0
        self.odomOld.pose.pose.position.x = initialX
        self.odomOld.pose.pose.position.y = initialY
        self.odomOld.pose.pose.orientation.z = initialTheta

        # Publishers and Subscribers
        self.subForRightCounts = self.create_subscription(Int16, 'right_ticks', self.calc_right, 10)
        self.subForLeftCounts = self.create_subscription(Int16, 'left_ticks', self.calc_left, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        timer_frequency = 5.0  # 5Hz
        self.timer = self.create_timer(1.0 / timer_frequency, self.publish_data)

    def calc_left(self, msg):
        global distanceLeft, lastCountL
        if msg.data != 0 and lastCountL != 0:
            leftTicks = msg.data - lastCountL
            if leftTicks > 10000:
                leftTicks = 0 - (65535 - leftTicks)
            elif leftTicks < -10000:
                leftTicks = 65535 - leftTicks
            distanceLeft = leftTicks / TICKS_PER_METER
        lastCountL = msg.data

    def calc_right(self, msg):
        global distanceRight, lastCountR
        if msg.data != 0 and lastCountR != 0:
            rightTicks = msg.data - lastCountR
            if rightTicks > 10000:
                rightTicks = 0 - (65535 - rightTicks)
            elif rightTicks < -10000:
                rightTicks = 65535 - rightTicks
            distanceRight = rightTicks / TICKS_PER_METER
        lastCountR = msg.data

    def publish_quat(self):
        q = tf2_ros.Quaternion()
        q.setRPY(0, 0, self.odomNew.pose.pose.orientation.z)

        quatOdom = Odometry()
        quatOdom.header.stamp = self.odomNew.header.stamp
        quatOdom.header.frame_id = "odom"
        quatOdom.child_frame_id = "base_link"
        quatOdom.pose.pose.position.x = self.odomNew.pose.pose.position.x
        quatOdom.pose.pose.position.y = self.odomNew.pose.pose.position.y
        quatOdom.pose.pose.position.z = self.odomNew.pose.pose.position.z
        quatOdom.pose.pose.orientation.x = q.x
        quatOdom.pose.pose.orientation.y = q.y
        quatOdom.pose.pose.orientation.z = q.z
        quatOdom.pose.pose.orientation.w = q.w
        quatOdom.twist.twist = self.odomNew.twist.twist

        for i in range(36):
            if i in [0, 7, 14]:
                quatOdom.pose.covariance[i] = .01
            elif i in [21, 28, 35]:
                quatOdom.pose.covariance[i] += 0.1
            else:
                quatOdom.pose.covariance[i] = 0

        self.odom_pub.publish(quatOdom)

    def update_odom(self):
        global distanceLeft, distanceRight

        cycleDistance = (distanceRight + distanceLeft) / 2
        cycleAngle = math.asin((distanceRight - distanceLeft) / WHEEL_BASE)
        avgAngle = cycleAngle / 2 + self.odomOld.pose.pose.orientation.z

        if avgAngle > PI:
            avgAngle -= 2 * PI
        elif avgAngle < -PI:
            avgAngle += 2 * PI

        self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x + math.cos(avgAngle) * cycleDistance
        self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y + math.sin(avgAngle) * cycleDistance
        self.odomNew.pose.pose.orientation.z = cycleAngle + self.odomOld.pose.pose.orientation.z

        if math.isnan(self.odomNew.pose.pose.position.x) or math.isnan(self.odomNew.pose.pose.position.y) or math.isnan(
                self.odomNew.pose.pose.position.z):
            self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x
            self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y
            self.odomNew.pose.pose.orientation.z = self.odomOld.pose.pose.orientation.z

        if self.odomNew.pose.pose.orientation.z > PI:
            self.odomNew.pose.pose.orientation.z -= 2 * PI
        elif self.odomNew.pose.pose.orientation.z < -PI:
            self.odomNew.pose.pose.orientation.z += 2 * PI

        self.odomNew.header.stamp = self.get_clock().now().to_msg()
        delta_time = (self.odomNew.header.stamp.sec + self.odomNew.header.stamp.nanosec * 1e-9) - (
                    self.odomOld.header.stamp.sec + self.odomOld.header.stamp.nanosec * 1e-9)
        self.odomNew.twist.twist.linear.x = cycleDistance / delta_time
        self.odomNew.twist.twist.angular.z = cycleAngle / delta_time
        self.odomOld = self.odomNew

    def publish_data(self):
        self.update_odom()
        self.publish_quat()


def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)

    # Destroy the node explicitly
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
