import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ignore_hardware = LaunchConfiguration('ignore_hardware')

    declare_ignore_hardware_cmd = DeclareLaunchArgument(
        'ignore_hardware',
        default_value='false',
        description='Jika true, tidak akan menyalakan node hardware untuk testing'
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_base_laser',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
    )
    
    fake_odom_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        condition=IfCondition(ignore_hardware)
    )

    fake_map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(ignore_hardware)
    )

    communication_node = Node(
        package='robot_base',
        executable='communication_node',
        name='communication_node',
        output='screen',
        condition=UnlessCondition(ignore_hardware)
    )

    odometry_node = Node(
        package='robot_base',
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )
    
    fsm_node = Node(
        package='robot_base',
        executable='fsm_node',
        name='fsm_node',
        output='screen',
        parameters=[{'bypass_health_check': ignore_hardware}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_ignore_hardware_cmd)
    ld.add_action(static_tf_node)
    ld.add_action(fake_odom_tf_node)
    ld.add_action(fake_map_tf_node)
    ld.add_action(communication_node)
    ld.add_action(odometry_node)
    ld.add_action(fsm_node)

    return ld
