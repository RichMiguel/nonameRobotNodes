import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_command = get_package_share_directory('robot_command')
    default_config = os.path.join(pkg_command, 'config', 'command_config.yaml')

    tcp_port = LaunchConfiguration('tcp_port')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_tcp_port_cmd = DeclareLaunchArgument(
        'tcp_port',
        default_value='5000',
        description='Port TCP socket untuk menerima perintah dari Voice Client / Dashboard'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Lokasi file konfigurasi command_config.yaml'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Gunakan waktu simulasi Gazebo'
    )

    command_node = Node(
        package='robot_command',
        executable='command_node',
        name='robot_command_node',
        output='screen',
        parameters=[{
            'tcp_port': tcp_port,
            'config_file': config_file,
            'use_sim_time': use_sim_time
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_tcp_port_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(command_node)

    return ld
