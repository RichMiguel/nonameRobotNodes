import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    ignore_hardware = LaunchConfiguration('ignore_hardware')
    use_gazebo = LaunchConfiguration('use_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_ignore_hardware_cmd = DeclareLaunchArgument(
        'ignore_hardware',
        default_value='false',
        description='Jika true, tidak akan menyalakan node hardware untuk testing'
    )
    
    declare_use_gazebo_cmd = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Jika true, kita berada dalam Gazebo (TF odom dipublish oleh Gazebo)'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Gunakan waktu simulasi jika berjalan di Gazebo'
    )

    # Hanya jalankan static tf base_link->laser jika BUKAN di Gazebo (karena URDF Gazebo sudah mempublishnya)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_base_laser',
        arguments=['0', '0', '0.2', '3.14159265', '0', '0', 'base_link', 'laser'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_gazebo)
    )
    
    # Fake Odom & Map hanya dijalankan jika hardware di-ignore DAN BUKAN di Gazebo (mode testing laptop murni)
    fake_odom_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", ignore_hardware, "' == 'true' and '", use_gazebo, "' == 'false'"]))
    )

    fake_map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", ignore_hardware, "' == 'true' and '", use_gazebo, "' == 'false'"]))
    )

    communication_node = Node(
        package='robot_base',
        executable='communication_node',
        name='communication_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(ignore_hardware)
    )

    # Odometry node juga dimatikan jika di Gazebo karena Planar Move Gazebo sudah mengeluarkan Odom TF & Topik
    odometry_node = Node(
        package='robot_base',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_gazebo)
    )
    
    fsm_node = Node(
        package='robot_base',
        executable='fsm_node',
        name='fsm_node',
        output='screen',
        parameters=[{
            'bypass_health_check': ignore_hardware,
            'use_sim_time': use_sim_time
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_ignore_hardware_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(static_tf_node)
    ld.add_action(fake_odom_tf_node)
    ld.add_action(fake_map_tf_node)
    ld.add_action(communication_node)
    ld.add_action(odometry_node)
    ld.add_action(fsm_node)

    return ld
