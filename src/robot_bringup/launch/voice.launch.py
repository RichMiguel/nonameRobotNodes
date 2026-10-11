from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node untuk Voice Command Parser
    voice_parser_node = Node(
        package='robot_voice_nav',
        executable='voice_cmd_parser',
        name='voice_cmd_parser_node',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(voice_parser_node)
    
    return ld
