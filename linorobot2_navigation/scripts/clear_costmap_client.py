import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class ClearCostmapClient(Node):
    def __init__(self):
        super().__init__('clear_costmap_client')
        self.client = self.create_client(Empty, 'clear_entirely_local_costmap')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.clear_costmap()

    def clear_costmap(self):
        request = Empty.Request()
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Costmap cleared successfully')
        else:
            self.get_logger().error('Failed to clear costmap')

def main(args=None):
    rclpy.init(args=args)
    node = ClearCostmapClient()
    rclpy.shutdown()

if __name__ == '__main__':
    main()