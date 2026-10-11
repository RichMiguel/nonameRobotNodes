import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ignore_hardware = LaunchConfiguration('ignore_hardware')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')

    declare_ignore_hardware_cmd = DeclareLaunchArgument(
        'ignore_hardware',
        default_value='false',
        description='Jika true, tidak menyalakan lidar'
    )
    
    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port untuk RPLidar (contoh: /dev/ttyUSB0 atau /dev/ttyUSB1)'
    )
    
    declare_serial_baudrate_cmd = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Baudrate RPLidar (115200 untuk A1 lama, 256000 untuk A1M8 baru)'
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')]),
        launch_arguments={
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate
        }.items(),
        condition=UnlessCondition(ignore_hardware)
    )

    ld = LaunchDescription()
    ld.add_action(declare_ignore_hardware_cmd)
    ld.add_action(declare_serial_port_cmd)
    ld.add_action(declare_serial_baudrate_cmd)
    ld.add_action(sllidar_launch)

    return ld
