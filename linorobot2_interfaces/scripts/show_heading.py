import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, Imu
from nav_msgs.msg import Odometry

# for ROS2 Humble subscribe to /imu/mag topic and log the heading angle in degrees only once every second
class ShowHeading(Node):
    def __init__(self):
        super().__init__('show_heading')
        self.mag_subscription = self.create_subscription(
            MagneticField,
            '/imu/mag',
            self.mag_callback,
            10
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.print_heading = 0

    def mag_callback(self, msg):
        if self.print_heading != 0:
            return
        heading = self.calculate_heading(msg)
        self.get_logger().info(f"Heading angle (from mag): {heading} degrees")
        self.print_heading = 1

    def imu_callback(self, msg):
        if self.print_heading != 1:
            return
        heading = self.calculate_heading_from_imu(msg)
        self.get_logger().info(f"Heading angle (from imu): {heading} degrees")
        self.print_heading = 2

    def odom_callback(self, msg):
        if self.print_heading != 2:
            return
        heading = self.extract_yaw_from_orientation(msg)
        self.get_logger().info(f"Heading angle (from odom): {heading} degrees")
        self.print_heading = 3

    def timer_callback(self):
        self.print_heading = 0

    def calculate_heading(self, msg):
        heading = 180 - math.atan2(msg.magnetic_field.y, msg.magnetic_field.x) * 180 / math.pi - 90
        if heading > 180:
            heading -= 360
        return heading

    def calculate_heading_from_imu(self, msg):
        quaternion = msg.orientation
        euler = self.quaternion_to_euler(quaternion)
        heading = euler[2] * 180 / math.pi
        return heading

    def extract_yaw_from_orientation(self, msg):
        quaternion = msg.pose.pose.orientation
        euler = self.quaternion_to_euler(quaternion)
        heading = euler[2] * 180 / math.pi
        return heading

    def quaternion_to_euler(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return [roll, pitch, yaw]

def main(args=None):
    rclpy.init(args=args)
    show_heading = ShowHeading()
    rclpy.spin(show_heading)
    show_heading.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
