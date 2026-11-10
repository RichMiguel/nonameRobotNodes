from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node Robot Command Coordinator (sebelumnya voice_cmd_parser)
    command_node = Node(
        package='robot_command',
        executable='command_node',
        name='robot_command_node',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(command_node)
    
    return ld
