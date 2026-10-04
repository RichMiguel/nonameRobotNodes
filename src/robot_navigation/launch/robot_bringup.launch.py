import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Deklarasi Parameter Launch
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        # Nanti arahkan file default ini ke peta asli Anda
        default_value=os.path.join(get_package_share_directory('robot_navigation'), 'maps', 'peta_ruangan.yaml'),
        description='Lokasi file peta .yaml')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('robot_navigation'), 'config', 'nav2_params.yaml'),
        description='Lokasi file parameter konfigurasi Nav2')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Gunakan waktu simulasi (Gazebo)')

    # 2. Menjalankan Node Kustom (Komunikasi, Odometri, FSM)
    communication_node = Node(
        package='base_controller',
        executable='communication_node',
        name='communication_node',
        output='screen'
        # Anda bisa menambahkan parameter port serial di sini jika diperlukan
        # parameters=[{'port': '/dev/ttyUSB0'}]
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
        output='screen'
    )

    # 3. Menjalankan RPLidar A1
    # Asumsi Anda menggunakan package standar sllidar_ros2
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')]),
        launch_arguments={'serial_port': '/dev/ttyUSB1'}.items() # Pastikan Lidar tidak bertabrakan port dengan Arduino
    )

    # 4. Menjalankan Master Nav2 (Mencakup Map Server, AMCL, Planner, Controller, BT, Costmaps)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file}.items()
    )

    # 5. Gabungkan seluruhnya ke dalam LaunchDescription
    ld = LaunchDescription()
    
    # Daftarkan Argumen
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Daftarkan Node Hardware & Konverter
    ld.add_action(communication_node)
    ld.add_action(odometry_node)
    ld.add_action(sllidar_launch)
    
    # Daftarkan Sistem Cerdas
    ld.add_action(nav2_bringup_launch)
    ld.add_action(fsm_node) # Dijalankan terakhir untuk mengawasi sistem

    return ld
