import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    als_dir = get_package_share_directory('als_ros2')

    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    als_params_file = LaunchConfiguration('als_params_file')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'lab_map.yaml'),
        description='Lokasi file peta .yaml'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Gunakan waktu simulasi (Gazebo)'
    )
    declare_als_params_cmd = DeclareLaunchArgument(
        'als_params_file',
        default_value=os.path.join(bringup_dir, 'config', 'als_mcl_params.yaml'),
        description='Lokasi file parameter konfigurasi als_ros2 mcl'
    )
    declare_initial_x_cmd = DeclareLaunchArgument(
        'initial_pose_x', default_value='2.45', description='Initial X pose (m)'
    )
    declare_initial_y_cmd = DeclareLaunchArgument(
        'initial_pose_y', default_value='1.8', description='Initial Y pose (m)'
    )
    declare_initial_yaw_cmd = DeclareLaunchArgument(
        'initial_pose_yaw', default_value='0.0', description='Initial Yaw pose (deg)'
    )

    mae_classifier_dir = os.path.join(als_dir, 'classifiers', 'MAE', 'default') + '/'

    # 1. Nav2 Map Server untuk menyediakan /map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # 2. ALS ROS 2 MCL Node (pengganti AMCL)
    als_mcl_node = Node(
        package='als_ros2',
        executable='mcl',
        name='mcl',
        output='screen',
        parameters=[
            os.path.join(bringup_dir, 'config', 'als_mcl_params.yaml'),
            als_params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'mae_classifier_dir': mae_classifier_dir,
                'initial_pose_x': ParameterValue(initial_pose_x, value_type=float),
                'initial_pose_y': ParameterValue(initial_pose_y, value_type=float),
                'initial_pose_yaw': ParameterValue(initial_pose_yaw, value_type=float),
            }
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_als_params_cmd)
    ld.add_action(declare_initial_x_cmd)
    ld.add_action(declare_initial_y_cmd)
    ld.add_action(declare_initial_yaw_cmd)

    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(als_mcl_node)

    return ld
