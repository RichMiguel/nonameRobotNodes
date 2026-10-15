from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    run_odometry = LaunchConfiguration('run_odometry')

    # Deklarasi argumen port dan baudrate (default ke /dev/ttyACM0 untuk Arduino Mega)
    declare_port_cmd = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Port serial Arduino Mega (contoh: /dev/ttyACM0 atau /dev/ttyUSB0)'
    )

    declare_baudrate_cmd = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate komunikasi serial Arduino'
    )

    declare_run_odometry_cmd = DeclareLaunchArgument(
        'run_odometry',
        default_value='true',
        description='Jalankan odometry_node untuk mengintegrasikan /raw_odom_vel menjadi /odom'
    )

    # 1. Node komunikasi ROS2 <-> Arduino Mega
    communication_node = Node(
        package='robot_base',
        executable='communication_node',
        name='communication_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': port,
            'baudrate': baudrate
        }]
    )

    # 2. Node odometri (menghitung X, Y, Yaw dari raw_odom_vel dan publish /odom + TF)
    odometry_node = Node(
        package='robot_base',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(run_odometry)
    )

    ld = LaunchDescription()
    ld.add_action(declare_port_cmd)
    ld.add_action(declare_baudrate_cmd)
    ld.add_action(declare_run_odometry_cmd)
    ld.add_action(communication_node)
    ld.add_action(odometry_node)

    return ld
