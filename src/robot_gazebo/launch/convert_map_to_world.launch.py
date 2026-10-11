import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_pkg = get_package_share_directory('robot_gazebo')
    bringup_pkg = get_package_share_directory('robot_bringup')

    default_map = os.path.join(bringup_pkg, 'maps', 'lab_map.yaml')
    default_world = os.path.join(gazebo_pkg, 'worlds', 'lab_map.world')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path ke file peta .yaml yang ingin dikonversi'
    )
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path output file .world'
    )
    declare_wall_height_cmd = DeclareLaunchArgument(
        'wall_height',
        default_value='2.0',
        description='Tinggi dinding (meter)'
    )
    declare_wall_thickness_cmd = DeclareLaunchArgument(
        'wall_thickness',
        default_value='0.05',
        description='Ketebalan dinding (meter)'
    )

    map_arg = LaunchConfiguration('map')
    world_arg = LaunchConfiguration('world')
    height_arg = LaunchConfiguration('wall_height')
    thickness_arg = LaunchConfiguration('wall_thickness')

    pipeline_script = os.path.join(gazebo_pkg, 'scripts', 'map_to_world_pipeline.py')

    run_pipeline = ExecuteProcess(
        cmd=[
            'python3', pipeline_script,
            '--map', map_arg,
            '--output', world_arg,
            '--wall-height', height_arg,
            '--wall-thickness', thickness_arg
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_world_cmd,
        declare_wall_height_cmd,
        declare_wall_thickness_cmd,
        run_pipeline
    ])
