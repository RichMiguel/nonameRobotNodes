import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Deklarasi Parameter Launch
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ignore_hardware = LaunchConfiguration('ignore_hardware')
    use_gazebo = LaunchConfiguration('use_gazebo')

    bringup_dir = get_package_share_directory('robot_bringup')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'lab_map.yaml'),
        description='Lokasi file peta .yaml')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Lokasi file parameter konfigurasi Nav2')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Gunakan waktu simulasi (Gazebo)')
        
    declare_ignore_hardware_cmd = DeclareLaunchArgument(
        'ignore_hardware',
        default_value='false',
        description='Jika true, tidak akan menyalakan node hardware (lidar & arduino) untuk keperluan testing')

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Menandakan bahwa sistem dijalankan dari Gazebo')

    # 2. Include Base Launch (TFs, Komunikasi, Odometri, FSM)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_dir, 'launch', 'base.launch.py')]),
        launch_arguments={
            'ignore_hardware': ignore_hardware, 
            'use_gazebo': use_gazebo,
            'use_sim_time': use_sim_time
        }.items()
    )

    declare_lidar_port_cmd = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port untuk RPLidar (contoh: /dev/ttyUSB0 atau /dev/ttyUSB1)')

    declare_lidar_baudrate_cmd = DeclareLaunchArgument(
        'lidar_baudrate',
        default_value='115200',
        description='Baudrate untuk RPLidar (115200 atau 256000)')

    declare_run_nav2_cmd = DeclareLaunchArgument(
        'run_nav2',
        default_value='true',
        description='Apakah akan menjalankan Nav2 Stack')

    declare_run_slam_cmd = DeclareLaunchArgument(
        'run_slam',
        default_value='false',
        description='Apakah akan menjalankan SLAM Toolbox (Pemetaan)')
        
    declare_use_voice_cmd = DeclareLaunchArgument(
        'use_voice',
        default_value='false',
        description='Apakah akan menjalankan modul penerima Voice Command (Socket server)')

    run_nav2 = LaunchConfiguration('run_nav2')
    run_slam = LaunchConfiguration('run_slam')
    use_voice = LaunchConfiguration('use_voice')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')

    # 3. Include Lidar Launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_dir, 'launch', 'lidar.launch.py')]),
        launch_arguments={
            'ignore_hardware': ignore_hardware,
            'serial_port': lidar_port,
            'serial_baudrate': lidar_baudrate
        }.items()
    )
    # 4. AMCL dan Map Server (Jalan jika run_slam = false)
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file}.items(),
        condition=__import__('launch').conditions.UnlessCondition(run_slam)
    )

    # 5. Nav2 Planner & Controller (Jalan jika run_nav2 = true)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_velocity_smoother': 'true',
            'params_file': params_file}.items(),
        condition=__import__('launch').conditions.IfCondition(run_nav2)
    )

    # 6. Menjalankan SLAM Toolbox (Jalan jika run_slam = true)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_dir, 'launch', 'slam_mapping.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=__import__('launch').conditions.IfCondition(run_slam)
    )

    # 7. Voice Command Node
    voice_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_dir, 'launch', 'voice.launch.py')]),
        condition=__import__('launch').conditions.IfCondition(use_voice)
    )

    # 8. Gabungkan seluruhnya ke dalam LaunchDescription
    ld = LaunchDescription()
    
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_ignore_hardware_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_run_nav2_cmd)
    ld.add_action(declare_run_slam_cmd)
    ld.add_action(declare_use_voice_cmd)
    ld.add_action(declare_lidar_port_cmd)
    ld.add_action(declare_lidar_baudrate_cmd)
    
    ld.add_action(base_launch)
    ld.add_action(lidar_launch)
    ld.add_action(amcl_launch)
    ld.add_action(navigation_launch)
    ld.add_action(slam_launch)
    ld.add_action(voice_launch)

    return ld
