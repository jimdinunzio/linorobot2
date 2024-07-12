from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declare_mag_bias_arg = DeclareLaunchArgument(
        'mag_bias',
        default_value='0.0,0.0,0.0',
        description='Magnetometer bias values.'
    )

    declare_mag_scale_arg = DeclareLaunchArgument(
        'mag_scale',
        default_value='0.0,0.0,0.0',
        description='Magnetometer scale values.'
    )

    declare_mag_calib_file_arg = DeclareLaunchArgument(
        'calib_file',
        default_value='',
        description='Path to the YAML file containing magnetometer calibration data.'
    )

    declare_mag_init_arg = DeclareLaunchArgument(
        'init',
        default_value='False',
        description='If true, run calibration and write new values to calib_file.'
    )

    calibrate_node = Node(
        package='linorobot2_interfaces',
        executable='calibrate_mag.py',
        parameters=[
            {'init': LaunchConfiguration('init')},
            {'mag_bias': LaunchConfiguration('mag_bias')},
            {'mag_scale': LaunchConfiguration('mag_scale')},
            {'calib_file': LaunchConfiguration('calib_file')}
        ]
    )

    return LaunchDescription([
        declare_mag_init_arg,
        declare_mag_bias_arg,
        declare_mag_scale_arg,
        declare_mag_calib_file_arg,
        calibrate_node,
    ])
if __name__ == '__main__':
    generate_launch_description()