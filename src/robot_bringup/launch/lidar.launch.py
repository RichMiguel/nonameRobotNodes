import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ignore_hardware = LaunchConfiguration('ignore_hardware')

    declare_ignore_hardware_cmd = DeclareLaunchArgument(
        'ignore_hardware',
        default_value='false',
        description='Jika true, tidak menyalakan lidar'
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')]),
        launch_arguments={'serial_port': '/dev/ttyUSB1'}.items(),
        condition=UnlessCondition(ignore_hardware)
    )

    ld = LaunchDescription()
    ld.add_action(declare_ignore_hardware_cmd)
    ld.add_action(sllidar_launch)

    return ld
