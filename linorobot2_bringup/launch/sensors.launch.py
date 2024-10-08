# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def generate_launch_description():
    laser_sensor_name = os.getenv('LINOROBOT2_LASER_SENSOR', '')
    depth_sensor_name = os.getenv('LINOROBOT2_DEPTH_SENSOR', '')
    
    fake_laser_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'config', 'fake_laser.yaml']
    )

    #indices
    #0 - depth topic (str)
    #1 - depth info topic (str)
    depth_topics = {
        '': ['', '', '', {}, '', ''],
        'realsense': ['/camera/depth/image_rect_raw', '/camera/depth/camera_info'],
        'astra': ['/depth/rgb/ir', '/camera_info'],
        'zed': ['/zed/depth/depth_registered', '/zed/depth/camera_info'],
        'zed2': ['/zed/depth/depth_registered', '/zed/depth/camera_info'],
        'zed2i': ['/zed/depth/depth_registered', '/zed/depth/camera_info'],
        'zedm': ['/zed/depth/depth_registered', '/zed/depth/camera_info'],
        'oakd': ['/right/image_rect', '/right/camera_info'],
        'oakdlite': ['/right/image_rect', '/right/camera_info'],
        'oakdpro': ['/right/image_rect', '/right/camera_info'],
    }

    point_cloud_topics = {
        '': '',
        'realsense': '/camera/depth/color/points',
        'astra': '/camera/depth/points',
        'zed': '/zed/point_cloud/cloud_registered',
        'zed2': '/zed/point_cloud/cloud_registered',
        'zed2i': '/zed/point_cloud/cloud_registered',
        'zedm': '/zed/point_cloud/cloud_registered',
        'oakd': '/stereo/points',
        'oakdlite': '/stereo/points',
        'oakdpro': '/stereo/points',
    }

    laser_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'lasers.launch.py']
    )

    depth_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'depth.launch.py']
    )

    gps_param = LaunchConfiguration('gps')

    declare_print_arg = DeclareLaunchArgument(
        'print',
        default_value='false',
        description='Enable printing'
    )
    print_param = LaunchConfiguration('print')

    declare_calib_data_file_arg = DeclareLaunchArgument(
        'calib_data_file',
        default_value=os.path.join(get_package_prefix('mpu9250'), 'config', 'calib_data.json'),
        description='Path to the calibration data file'
    )
    calib_data_file_param = LaunchConfiguration('calib_data_file')

    declare_calibrate_arg = DeclareLaunchArgument(
        'calibrate',
        default_value='false',
        description='Do Gyro and Magnetometer calibration'
    )
    calibrate_param = LaunchConfiguration('calibrate')

    return LaunchDescription([
        GroupAction(
            actions=[
                SetRemap(src=point_cloud_topics[depth_sensor_name], dst='/camera/depth/color/points'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(depth_launch_path),
                    condition=IfCondition(PythonExpression(['"" != "', depth_sensor_name, '"'])),
                    launch_arguments={'sensor': depth_sensor_name}.items()   
                )
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_path),
            condition=IfCondition(PythonExpression(['"" != "', laser_sensor_name, '"'])),
            launch_arguments={
                'sensor': laser_sensor_name
            }.items()   
        ),
        Node(
            condition=IfCondition(PythonExpression(['"" != "', laser_sensor_name, '" and ', '"', laser_sensor_name, '" in "', str(list(depth_topics.keys())[1:]), '"'])),
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            remappings=[('depth', depth_topics[depth_sensor_name][0]),
                        ('depth_camera_info', depth_topics[depth_sensor_name][1])],
            parameters=[fake_laser_config_path]
        ),

        declare_print_arg,
        declare_calibrate_arg,
        declare_calib_data_file_arg,
        Node(
            package="mpu9250",
            executable="mpu9250",
            name="mpu9250",
            parameters=[
                {"acceleration_scale": [1.0070642813137283, 1.0077919346121842, 0.979079278290781], 
                 "acceleration_bias": [0.30351858720785296, 0.03640315588498285, 0.014441728200428484],
                 "print": print_param,
                 "calibrate": calibrate_param,
                 "calib_data_file": calib_data_file_param,
                 "do_apply_ema": True,
                 "ema_alpha": 0.1}
            ],
        ),

        Node(
            condition=IfCondition(gps_param),
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('nmea_navsat_driver'), 'config', 'nmea_serial_driver.yaml')
            ]
        )
    ])

   