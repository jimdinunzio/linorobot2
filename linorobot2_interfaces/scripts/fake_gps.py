import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

#!/usr/bin/env python3

# This is a python script to publish the following GPS data to /fix topic using ROS2 libraries once per second repeatedly.
# header:
#   stamp:
#     sec: <current time>
#     nanosec: <current time>
#   frame_id: gps
# status:
#   status: 0
#   service: 1
# latitude: 33.653596666666665
# longitude: -117.86949
# altitude: -18.0
# position_covariance:
# - 31.359999999999996
# - 0.0
# - 0.0
# - 0.0
# - 31.359999999999996
# - 0.0
# - 0.0
# - 0.0
# - 501.75999999999993
# position_covariance_type: 1
# ---
class FakeGpsPublisher(Node):
    def __init__(self):
        super().__init__('fake_gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, '/fix', 10)
        self.timer_ = self.create_timer(1.0, self.publish_gps_data)

    def publish_gps_data(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = 0
        msg.status.service = 1
        msg.latitude = 33.653596666666665
        msg.longitude = -117.86949
        msg.altitude = -18.0
        msg.position_covariance = [31.36, 0.0, 0.0, 0.0, 31.36, 0.0, 0.0, 0.0, 501.76]
        msg.position_covariance_type = 1

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    fake_gps_publisher = FakeGpsPublisher()
    rclpy.spin(fake_gps_publisher)
    fake_gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()