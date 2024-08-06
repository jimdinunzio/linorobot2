import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, Imu
from nav_msgs.msg import Odometry
import math
from scipy.spatial.transform import Rotation as R

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
        self.timer = self.create_timer(1, self.timer_callback)
        self.print_heading = 0 
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = -9.81  

    def mag_callback(self, msg):
        if self.print_heading != 0:
            return
        heading = self.calculate_heading(msg)
        self.get_logger().info(f"-----------------------")
        self.get_logger().info(f"mag_x: {msg.magnetic_field.x}, mag_y: {msg.magnetic_field.y}, mag_z: {msg.magnetic_field.z}")
        self.get_logger().info(f"Heading angle (from mag): {heading} degrees\n")
        self.print_heading = 1

    def imu_callback(self, msg):
        self.accel_x = msg.linear_acceleration.x
        self.accel_y = msg.linear_acceleration.y
        self.accel_z = msg.linear_acceleration.z

        if self.print_heading != 1:
            return
        heading = self.calculate_heading_from_imu(msg)
        self.get_logger().info(f"Heading angle (from imu): {heading} degrees\n")
        self.print_heading = 2

    def odom_callback(self, msg):
        if self.print_heading != 2:
            return
        heading = self.extract_yaw_from_orientation(msg)
        self.get_logger().info(f"Heading angle (from odom): {heading} degrees\n")
        self.print_heading = 3

    def timer_callback(self):
        self.print_heading = 0

    # def calculate_heading(self, msg):
    #     vector_mag = [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z]
    #     vector_down = [self.accel_x, self.accel_y, self.accel_z]
    #     #self.get_logger().info(f"accel_x: {self.accel_x}, accel_y: {self.accel_y}, accel_z: {self.accel_z}")
    #     dot_product = sum(a * b for a, b in zip(vector_mag, vector_down))
    #     dot_product /= sum(a * b for a, b in zip(vector_down, vector_down))
    #     vector_north = [a - dot_product * b for a, b in zip(vector_mag, vector_down)]
    #     heading = math.atan2(vector_north[1], -vector_north[0]) * 180 / math.pi
    #     return heading

    def calculate_heading(self, msg):
        heading = math.atan2(msg.magnetic_field.x, msg.magnetic_field.y) * 180 / math.pi
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
        rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        angle = rotation.as_euler('xyz')
                                  
        # x = quaternion.x
        # y = quaternion.y
        # z = quaternion.z
        # w = quaternion.w

        # t0 = +2.0 * (w * x + y * z)
        # t1 = +1.0 - 2.0 * (x * x + y * y)
        # roll = math.atan2(t0, t1)

        # t2 = +2.0 * (w * y - z * x)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        # pitch = math.asin(t2)

        # t3 = +2.0 * (w * z + x * y)
        # t4 = +1.0 - 2.0 * (y * y + z * z)
        # yaw = math.atan2(t3, t4)

        # if yaw > math.pi:
        #     yaw -= 2 * math.pi

        #return [roll, pitch, yaw]
        return angle

def main(args=None):
    rclpy.init(args=args)
    show_heading = ShowHeading()
    try:
        rclpy.spin(show_heading)
    except KeyboardInterrupt:
        pass
    show_heading.destroy_node()

if __name__ == '__main__':
    main()
