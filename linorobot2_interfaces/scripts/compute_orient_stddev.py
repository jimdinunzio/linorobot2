import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import numpy as np
from scipy.spatial.transform import Rotation as R


class CollectOrientations(Node):
    def imu_callback(self, msg):
        quaternion = msg.orientation
        quaternion_np = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.quaternions.append(quaternion_np)
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(f"Received {self.count} orientation measurements...")
        if self.count == 1000:
            std_dev = self.calculate_std_dev()
            self.get_logger().info(f"Standard deviation of orientation measurements: {std_dev}")
            self.destroy_node()
            exit()

    def __init__(self):
        super().__init__('collect_orientations')
        self.quaternions = []
        imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
            )
        self.count = 0
        self.get_logger().info(f"Collecting orientations from Madgwick output on /imu/data topic...")

    def calculate_std_dev(self):
        # Assume `quaternions` is a 2D numpy array where each row is a quaternion
        # in the form (w, x, y, z).
        rotations = R.from_quat(self.quaternions)
        euler_angles = rotations.as_euler('xyz')
        # `euler_angles` now contains the roll, pitch, and yaw angles in radians.

        # Assume `euler_angles` is a 2D numpy array where each row is a measurement
        # and the columns correspond to roll, pitch, and yaw.

        # Calculate the standard deviation for each column (roll, pitch, yaw).
        std_devs = np.std(euler_angles, axis=0)
        # Take the average of the standard deviations.
        avg_std_dev = np.mean(std_devs)

        return avg_std_dev
def main(args=None):
    rclpy.init(args=args)
    collectOrientations = CollectOrientations()
    try:
        rclpy.spin(collectOrientations)
    except KeyboardInterrupt:
        pass
    collectOrientations.destroy_node()

if __name__ == '__main__':
    main()
