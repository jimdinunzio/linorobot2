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
    node.declare_parameter('init', False) # if true then run calibration and write new values to calib_file
    node.declare_parameter('mag_bias', '0.0,0.0,0.0')
    node.declare_parameter('mag_scale', '0.0,0.0,0.0')
    node.declare_parameter('calib_file', '')
    
    calib_file = node.get_parameter('calib_file').get_parameter_value().string_value
    has_calib_file = calib_file != ''
    run_calibration = node.get_parameter('init').get_parameter_value().bool_value

    # Load parameters from YAML file
    if not run_calibration:
        if has_calib_file:
            mag_bias, mag_scale = load_params_from_yaml(node.get_parameter('calib_file').get_parameter_value().string_value)
            node.get_logger().info(f"calib_file: {node.get_parameter('calib_file').get_parameter_value().string_value}")
    else:
        node.get_logger().info("Running calibration...")
    
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
            if run_calibration and has_calib_file:
                with open(calib_file, 'w') as file:
                    yaml.dump({'mag_bias': [mag_bias.x, mag_bias.y, mag_bias.z], 'mag_scale': [mag_scale.x, mag_scale.y, mag_scale.z]}, file)
                    node.get_logger().info(f"Calibration data written to {calib_file}")
            # Shutdown the node after receiving the response
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            node.get_logger().error(f"Service call failed: {e}")

    send_request()
    if run_calibration:
        node.get_logger().info("Start rotating the robot in all directions to calibrate the magnetometer and continue for 20 seconds.")
    rclpy.spin(node)

if __name__ == "__main__":
    main()
