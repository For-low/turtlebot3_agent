from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_agent',
            executable='agent_node', # Matches the entry point in setup.py
            name='turtlebot3_agent_node',
            output='screen',
            # No emulate_tty=True - seemed problematic
        )
    ])