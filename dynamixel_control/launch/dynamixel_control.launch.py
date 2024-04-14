from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamixel_control',
            executable='dynamixel_node',
            name='dynamixel_node',
            output='screen',
            parameters=[
                {'startup': True},
                {'device_name': '/dev/ttyUSB0'},
                {'baudrate': 57600},
                {'protocol_version': 1.0},
                {'id_1': 1},
                {'id_2': 2},
            ]
        )
    ])