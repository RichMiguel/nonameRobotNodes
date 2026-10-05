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

    bringup_dir = get_package_share_directory('robot_bringup')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'map0.yaml'),
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

    # 2. Include Base Launch (TFs, Komunikasi, Odometri, FSM)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_dir, 'launch', 'base.launch.py')]),
        launch_arguments={'ignore_hardware': ignore_hardware}.items()
    )

    # 3. Include Lidar Launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_dir, 'launch', 'lidar.launch.py')]),
        launch_arguments={'ignore_hardware': ignore_hardware}.items()
    )

    # 4. Menjalankan Master Nav2
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
    
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_ignore_hardware_cmd)
    
    ld.add_action(base_launch)
    ld.add_action(lidar_launch)
    ld.add_action(nav2_bringup_launch)

    return ld
