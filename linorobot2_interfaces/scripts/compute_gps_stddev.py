import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import numpy as np
import logging

# Create a logger
logger = logging.getLogger(__name__)

# write a script that computes the variance of the GPS coordinates.
# The script should subscribe to the /fix topic and collect the GPS coordinates
# in a list. Once the list contains n GPS coordinates, the script should
# calculate the variance of the latitude, longitude, and altitude.
# The script should print the variances to the console in the form of the position_covariance matrix.

n = 100 # Number of GPS coordinates to collect before calculating variance

class ComputeGPSStdDev(Node):
    def gps_callback(self, data):
        self.gps_coordinates.append(data)
        self.count += 1

        if self.count % 10 == 0:
            self.get_logger().info("Collected {} gps coordinates".format(len(self.gps_coordinates)))

        if self.count == self.n:
            self.compute_variance()
    
    def __init__(self):
        super().__init__('compute_gps_stddev')
        self.get_logger().info("Collecting GPS coordinates from /fix topic...")
        self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.n = n  # Number of GPS coordinates to collect before calculating variance
        self.gps_coordinates = []
        self.count = 0

    def compute_variance(self):
        latitudes = [coord.latitude for coord in self.gps_coordinates]
        longitudes = [coord.longitude for coord in self.gps_coordinates]
        altitudes = [coord.altitude for coord in self.gps_coordinates]

        latitude_variance = self.calculate_variance(latitudes)
        longitude_variance = self.calculate_variance(longitudes)
        altitude_variance = self.calculate_variance(altitudes)

        self.get_logger().info("Position Covariance:")
        self.get_logger().info("lattatude variance: {}".format(latitude_variance))
        self.get_logger().info("longitude variance: {}".format(longitude_variance))
        self.get_logger().info("altitude variance: {}".format(altitude_variance))

        self.get_logger().info("[{},0.0,0.0;0.0,{},0.0;0.0,0.0,{}]"
                         .format(latitude_variance, longitude_variance, altitude_variance))
        exit()


    def calculate_variance(self, data):
        variance = np.var(data)
        return variance

def main(args=None):
    rclpy.init(args=args)
    compute_gps_stddev = ComputeGPSStdDev()
    rclpy.spin(compute_gps_stddev)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
