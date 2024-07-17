import rclpy
from rclpy.node import Node
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu, MagneticField

MAG_SAMPLES = 1000
ORIENT_SAMPLES = 1000

class CollectMagneticField(Node):
    def mag_callback(self, msg):
        mag_field = msg.magnetic_field
        mag_field_np = np.array([mag_field.x, mag_field.y, mag_field.z])
        self.mag_fields.append(mag_field_np)
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(f"Received {self.count} magnetic field measurements...")
        if self.count == MAG_SAMPLES:
            variances = self.calculate_variances()
            self.get_logger().info(f"Variance of magnetic field measurements: x: {variances[0]}, y: {variances[1]}, z: {variances[2]}")
            self.done = True

    def __init__(self):
        super().__init__('collect_magnetic_field')
        self.mag_fields = []
        mag_subscription = self.create_subscription(
            MagneticField,
            '/imu/mag',
            self.mag_callback,
            10
            )
        self.count = 0
        self.get_logger().info(f"Collecting magnetic fields from /imu/mag topic...")
        self.done = False
        
    def calculate_variances(self):
        mag_fields = np.array(self.mag_fields)
        variances = np.var(mag_fields, axis=0)
        return variances
    
class CollectOrientations(Node):
    def imu_callback(self, msg):
        quaternion = msg.orientation
        quaternion_np = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.quaternions.append(quaternion_np)
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(f"Received {self.count} orientation measurements...")
        if self.count == ORIENT_SAMPLES:
            std_deviations = self.calculate_std_deviations()
            average_std_deviation = np.mean(std_deviations)
            self.get_logger().info(f"Average standard deviation of orientation measurements: {average_std_deviation}")
            self.done = True

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
        self.done = False

    def calculate_std_deviations(self):
        rotations = R.from_quat(self.quaternions)
        euler_angles = rotations.as_euler('xyz')
        std_deviations = np.std(euler_angles, axis=0)
        return std_deviations
    
def main(args=None):
    rclpy.init(args=args)
    collectOrientations = CollectOrientations()
    collectMagneticField = CollectMagneticField()
    executor = rclpy.executors.MultiThreadedExecutor()

    executor.add_node(collectOrientations)
    executor.add_node(collectMagneticField)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=1.0)
            if collectOrientations.done == True and collectMagneticField.done == True:
                rclpy.shutdown()
                return
    except KeyboardInterrupt:
        pass
        
if __name__ == '__main__':
    main()
