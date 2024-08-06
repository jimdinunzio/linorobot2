import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class OrientationSubscriber(Node):
    def __init__(self):
        super().__init__('orientation_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.subscription

    def imu_callback(self, msg):
        # Convert orientation quaternions to pitch, yaw, and roll
        quaternion = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)

        # Convert roll, pitch, and yaw to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # Print out the result
        self.get_logger().info(f"Pitch: {pitch_deg}, Yaw: {yaw_deg}, Roll: {roll_deg}")

    def quaternion_to_euler(self, x, y, z, w):
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

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    orientation_subscriber = OrientationSubscriber()
    rclpy.spin(orientation_subscriber)
    orientation_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()