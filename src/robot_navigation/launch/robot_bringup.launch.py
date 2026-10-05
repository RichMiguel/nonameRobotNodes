import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Deklarasi Parameter Launch
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ignore_hardware = LaunchConfiguration('ignore_hardware')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('robot_navigation'), 'maps', 'map0.yaml'),
        description='Lokasi file peta .yaml')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('robot_navigation'), 'config', 'nav2_params.yaml'),
        description='Lokasi file parameter konfigurasi Nav2')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Gunakan waktu simulasi (Gazebo)')
        
    declare_ignore_hardware_cmd = DeclareLaunchArgument(
        'ignore_hardware',
        default_value='false',
        description='Jika true, tidak akan menyalakan node hardware (lidar & arduino) untuk keperluan testing')

    # 2. TF Statis (Menghubungkan base_link ke laser agar Lidar bisa dibaca RViz & Nav2)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_base_laser',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
    )
    
    # 2.5 TF Statis (Fake Odom) HANYA untuk mode bypass hardware
    fake_odom_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        condition=IfCondition(ignore_hardware)
    )

    # 2.6 TF Statis (Fake Map) HANYA untuk mode bypass hardware
    # Menggantikan peran AMCL yang tidak akan berjalan tanpa data Lidar
    fake_map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(ignore_hardware)
    )

    # 3. Menjalankan Node Kustom (Komunikasi, Odometri, FSM)
    communication_node = Node(
        package='base_controller',
        executable='communication_node',
        name='communication_node',
        output='screen',
        condition=UnlessCondition(ignore_hardware)
    )

    odometry_node = Node(
        package='base_controller',
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )
    
    fsm_node = Node(
        package='base_controller',
        executable='fsm_node',
        name='fsm_node',
        output='screen',
        parameters=[{'bypass_health_check': ignore_hardware}]
    )

    # 4. Menjalankan RPLidar A1 (Hanya jika ignore_hardware=false)
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')]),
        launch_arguments={'serial_port': '/dev/ttyUSB1'}.items(),
        condition=UnlessCondition(ignore_hardware)
    )

    # 5. Menjalankan Master Nav2 (Mencakup Map Server, AMCL, Planner, Controller, BT, Costmaps)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file}.items()
    )

    # 6. Gabungkan seluruhnya ke dalam LaunchDescription
    ld = LaunchDescription()
    
    # Daftarkan Argumen
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_ignore_hardware_cmd)
    
    # Daftarkan Node & Sistem
    ld.add_action(static_tf_node)
    ld.add_action(fake_odom_tf_node)
    ld.add_action(fake_map_tf_node)
    ld.add_action(communication_node)
    ld.add_action(odometry_node)
    ld.add_action(sllidar_launch)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(fsm_node)

    return ld
