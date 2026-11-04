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
    launch_base = LaunchConfiguration('launch_base')
    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')
    strafe_speed = LaunchConfiguration('strafe_speed')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ignore_hardware = LaunchConfiguration('ignore_hardware')
    use_gazebo = LaunchConfiguration('use_gazebo')

    # Deklarasi Argumen Launch
    declare_launch_base_cmd = DeclareLaunchArgument(
        'launch_base',
        default_value='true',
        description='Jalankan base hardware (komunikasi serial Arduino & odometri) bersama teleop. Set false jika base sudah berjalan di terminal lain.'
    )

    declare_linear_speed_cmd = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.3',
        description='Kecepatan translasi linear maju/mundur (m/s)'
    )

    declare_angular_speed_cmd = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.8',
        description='Kecepatan rotasi angular (rad/s)'
    )

    declare_strafe_speed_cmd = DeclareLaunchArgument(
        'strafe_speed',
        default_value='0.3',
        description='Kecepatan translasi lateral geser kiri/kanan (m/s)'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Gunakan waktu simulasi (true jika di Gazebo)'
    )

    declare_ignore_hardware_cmd = DeclareLaunchArgument(
        'ignore_hardware',
        default_value='false',
        description='Jika true, tidak menyalakan serial hardware Arduino (mode simulasi/testing laptop)'
    )

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='True jika berjalan di dalam Gazebo'
    )

    # 1. Base Hardware & Odometry (hanya jalan jika launch_base = true)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_dir, 'launch', 'base.launch.py')]),
        launch_arguments={
            'ignore_hardware': ignore_hardware,
            'use_gazebo': use_gazebo,
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(launch_base)
    )

    # 2. Node Keyboard Teleop WASD
    teleop_node = Node(
        package='robot_base',
        executable='teleop_wasd_node',
        name='teleop_wasd_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'linear_speed': linear_speed,
            'angular_speed': angular_speed,
            'strafe_speed': strafe_speed,
            'use_sim_time': use_sim_time
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_launch_base_cmd)
    ld.add_action(declare_linear_speed_cmd)
    ld.add_action(declare_angular_speed_cmd)
    ld.add_action(declare_strafe_speed_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_ignore_hardware_cmd)
    ld.add_action(declare_use_gazebo_cmd)

    ld.add_action(base_launch)
    ld.add_action(teleop_node)

    return ld
