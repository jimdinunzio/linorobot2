#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from linorobot2_interfaces.srv import CalibrateMag
import yaml

def get_vector3_param(node, param_name):
    if node.has_parameter(param_name):
        param_value = node.get_parameter(param_name).get_parameter_value().string_value
        x, y, z = map(float, param_value.split(','))
        return Vector3(x=x, y=y, z=z)

def load_params_from_yaml(file_path):
    with open(file_path, 'r') as file:
        params = yaml.safe_load(file)
        mag_bias = params.get('mag_bias')
        mag_scale = params.get('mag_scale')
        return Vector3(x=mag_bias[0], y=mag_bias[1], z=mag_bias[2]), Vector3(x=mag_scale[0], y=mag_scale[1], z=mag_scale[2])

def main(args=None):
    rclpy.init(args=args)
    node = Node("calibrate_node")
    
    mag_bias = None
    mag_scale = None

    # Declare parameters
    node.declare_parameter('mag_bias', '0.0,0.0,0.0')
    node.declare_parameter('mag_scale', '0.0,0.0,0.0')
    node.declare_parameter('calib_file', '')
    
    # Load parameters from YAML file
    if node.get_parameter('calib_file').get_parameter_value().string_value != '':
        mag_bias, mag_scale = load_params_from_yaml(node.get_parameter('calib_file').get_parameter_value().string_value)
        node.get_logger().info(f"calib_file: {node.get_parameter('calib_file').get_parameter_value().string_value}")

    client = node.create_client(CalibrateMag, "/calibrate_mag")
    request = CalibrateMag.Request()

    request.mag_bias = mag_bias if mag_bias else get_vector3_param(node, 'mag_bias')
    node.get_logger().info(f"Input Mag Bias: {request.mag_bias}")
    request.mag_scale = mag_scale if mag_scale else get_vector3_param(node, 'mag_scale')
    node.get_logger().info(f"Input Mag Scale: {request.mag_scale}")
    
    # Wait until the service is available
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')

    def send_request():
        future = client.call_async(request)
        future.add_done_callback(handle_response)

    def handle_response(future):
        try:
            response = future.result()
            mag_scale = response.mag_scale
            mag_bias = response.mag_bias
            node.get_logger().info(f"Mag Bias: {mag_bias}")
            node.get_logger().info(f"Mag Scale: {mag_scale}")
            # Shutdown the node after receiving the response
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            node.get_logger().error(f"Service call failed: {e}")

    send_request()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
