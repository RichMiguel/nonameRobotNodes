import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    ignore_hardware = LaunchConfiguration('ignore_hardware')
    use_gazebo = LaunchConfiguration('use_gazebo')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    slam_params_file = LaunchConfiguration('slam_params_file')
    run_teleop = LaunchConfiguration('run_teleop')

    # Deklarasi Argumen Launch
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Gunakan waktu simulasi (true jika di Gazebo)'
    )

    declare_ignore_hardware_cmd = DeclareLaunchArgument(
        'ignore_hardware',
        default_value='false',
        description='Jika true, tidak menyalakan hardware fisik (Arduino serial & LiDAR)'
    )

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='True jika robot berada di dalam simulasi Gazebo'
    )

    declare_lidar_port_cmd = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port untuk RPLidar (default: /dev/ttyUSB0)'
    )

    declare_lidar_baudrate_cmd = DeclareLaunchArgument(
        'lidar_baudrate',
        default_value='115200',
        description='Baudrate RPLidar (default: 115200)'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(bringup_dir, 'config', 'mapping_params.yaml'),
        description='Lokasi file parameter konfigurasi SLAM Toolbox'
    )

    declare_run_teleop_cmd = DeclareLaunchArgument(
        'run_teleop',
        default_value='false',
        description='Jalankan teleop_wasd_node di launch ini. Rekomendasi: biarkan false dan jalankan di terminal terpisah agar input keyboard lancar tanpa gangguan log.'
    )

    # 1. Base Hardware & Odometry (base.launch.py)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_dir, 'launch', 'base.launch.py')]),
        launch_arguments={
            'ignore_hardware': ignore_hardware,
            'use_gazebo': use_gazebo,
            'use_sim_time': use_sim_time
        }.items()
    )

    # 2. RPLiDAR A1 Sensor (lidar.launch.py)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_dir, 'launch', 'lidar.launch.py')]),
        launch_arguments={
            'ignore_hardware': ignore_hardware,
            'serial_port': lidar_port,
            'serial_baudrate': lidar_baudrate
        }.items()
    )

    # 3. SLAM Toolbox Node (Mapping Mode)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 4. Optional Teleop WASD Node (jika run_teleop:=true)
    teleop_node = Node(
        package='robot_base',
        executable='teleop_wasd_node',
        name='teleop_wasd_node',
        output='screen',
        condition=IfCondition(run_teleop)
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_ignore_hardware_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_lidar_port_cmd)
    ld.add_action(declare_lidar_baudrate_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_run_teleop_cmd)

    ld.add_action(base_launch)
    ld.add_action(lidar_launch)
    ld.add_action(slam_toolbox_node)
    ld.add_action(teleop_node)

    return ld
