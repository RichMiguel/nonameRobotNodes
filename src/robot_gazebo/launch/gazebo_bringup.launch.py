import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Paths
    gazebo_pkg = get_package_share_directory('robot_gazebo')
    bringup_pkg = get_package_share_directory('robot_bringup')
    
    # Declare Launch Arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(gazebo_pkg, 'worlds', 'lab_map.world'),
        description='Path ke file SDF / world Gazebo Harmonic'
    )
    declare_model_cmd = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(gazebo_pkg, 'urdf', 'xdrive.xacro'),
        description='Path ke file xacro model robot'
    )
    declare_run_nav2_cmd = DeclareLaunchArgument(
        'run_nav2', default_value='true', description='Jalankan Nav2 Stack di Gazebo')
    declare_run_slam_cmd = DeclareLaunchArgument(
        'run_slam', default_value='false', description='Jalankan SLAM Toolbox di Gazebo')
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=os.path.join(bringup_pkg, 'maps', 'lab_map.yaml'),
        description='Lokasi file peta yaml')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(bringup_pkg, 'config', 'nav2_params.yaml'),
        description='Lokasi file parameter konfigurasi Nav2')
    declare_use_voice_cmd = DeclareLaunchArgument(
        'use_voice', default_value='false', description='Jalankan modul penerima Voice Command (Socket server)')

    # Posisi spawn robot (koordinat world = koordinat map lab_map, origin [0,0])
    # Default: tengah ruangan bawah (dinding x 0.63-4.27, y 0.60-3.00)
    declare_spawn_x_cmd = DeclareLaunchArgument(
        'spawn_x', default_value='2.45', description='Posisi X spawn robot (meter)')
    declare_spawn_y_cmd = DeclareLaunchArgument(
        'spawn_y', default_value='1.8', description='Posisi Y spawn robot (meter)')
    declare_spawn_yaw_cmd = DeclareLaunchArgument(
        'spawn_yaw', default_value='0.0', description='Orientasi yaw spawn robot (radian)')

    world_file = LaunchConfiguration('world')
    model_file = LaunchConfiguration('model')
    run_nav2 = LaunchConfiguration('run_nav2')
    run_slam = LaunchConfiguration('run_slam')
    map_yaml_file = LaunchConfiguration('map')
    use_voice = LaunchConfiguration('use_voice')
    params_file = LaunchConfiguration('params_file')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    # Process Xacro (Default: xdrive.xacro)
    default_urdf = os.path.join(gazebo_pkg, 'urdf', 'xdrive.xacro')
    doc = xacro.process_file(default_urdf)
    robot_description_xml = doc.toxml()
    robot_description = {'robot_description': robot_description_xml}

    # Simpan URDF ke file agar bisa di-spawn oleh gz sim
    urdf_output_path = os.path.join(gazebo_pkg, 'urdf', 'xdrive.urdf')
    with open(urdf_output_path, 'w') as f:
        f.write(robot_description_xml)

    # 0. Bersihkan proses gz sim server zombie sebelum mulai agar state posisi & odom selalu reset ke 0
    pre_clean_process = ExecuteProcess(
        cmd=['pkill', '-9', '-f', 'gz sim server'],
        output='log'
    )

    # 1. Jalankan Gazebo Harmonic (gz sim)
    gz_sim_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # Handlers untuk membersihkan background proses saat shutdown / exit
    shutdown_clean_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(cmd=['pkill', '-9', '-f', 'gz sim server'], output='log')
            ]
        )
    )

    gz_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_sim_process,
            on_exit=[
                ExecuteProcess(cmd=['pkill', '-9', '-f', 'gz sim server'], output='log'),
                Shutdown(reason='Gazebo GUI ditutup oleh user')
            ]
        )
    )

    # 2. Robot State Publisher (Broadcasts URDF TFs: base_footprint -> base_link -> laser & wheels)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 3. Spawn Robot in Gazebo Harmonic via ros_gz_sim create (menggunakan file URDF)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'generated_world',
            '-file', urdf_output_path,
            '-name', 'xdrive',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', '0.08',
            '-Y', spawn_yaw
        ],
        output='screen'
    )

    # 4. Bridge antara ROS 2 dan Gazebo Harmonic (ros_gz_bridge)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/xdrive/joint/front_left_wheel_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/xdrive/joint/front_right_wheel_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/xdrive/joint/rear_left_wheel_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/xdrive/joint/rear_right_wheel_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen'
    )

    # 4b. Fallback static TF: menjamin frame LiDAR selalu terhubung ke robot TF tree (laser),
    # jika Gazebo menamai frame dengan format <model>/<link>/<sensor> ('xdrive/base_footprint/rplidar_sensor')
    static_lidar_fallback_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_lidar_fallback_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'laser', 'xdrive/base_footprint/rplidar_sensor'],
        parameters=[{'use_sim_time': True}]
    )

    # 4c. Node Kinematika Invers X-Drive (Menghitung kecepatan masing-masing roda dari cmd_vel)
    node_xdrive_kinematics = Node(
        package='robot_base',
        executable='xdrive_kinematics_node',
        name='xdrive_kinematics_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'wheel_radius': 0.05,
            'wheel_offset_x': 0.20,
            'wheel_offset_y': 0.20
        }]
    )

    # 5. Include Core Robot Bringup (Nav2, FSM, Maps) with Hardware Ignored
    robot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_pkg, 'launch', 'robot_bringup.launch.py')]),
        launch_arguments={
            'use_sim_time': 'true',
            'ignore_hardware': 'true',
            'use_gazebo': 'true',
            'run_nav2': run_nav2,
            'run_slam': run_slam,
            'map': map_yaml_file,
            'params_file': params_file,
            'use_voice': use_voice
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_run_nav2_cmd)
    ld.add_action(declare_run_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_voice_cmd)
    ld.add_action(declare_spawn_x_cmd)
    ld.add_action(declare_spawn_y_cmd)
    ld.add_action(declare_spawn_yaw_cmd)

    ld.add_action(pre_clean_process)
    ld.add_action(gz_sim_process)
    ld.add_action(shutdown_clean_handler)
    ld.add_action(gz_exit_handler)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(TimerAction(period=5.0, actions=[spawn_entity]))
    ld.add_action(ros_gz_bridge)
    ld.add_action(static_lidar_fallback_tf)
    ld.add_action(node_xdrive_kinematics)
    ld.add_action(robot_bringup_launch)

    return ld
