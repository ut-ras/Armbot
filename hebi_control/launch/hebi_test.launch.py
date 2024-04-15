from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hebi_control',
            executable='hebi_control_node',
            name='hebi_control_node',
            output='screen',
        ),
    ])