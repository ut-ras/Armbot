from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the path to the dynamixel_controllers.yaml file within your package
    hardware_config_file = PathJoinSubstitution([
        FindPackageShare('dynamixel_control'),  # replace 'dynamixel_control' with the name of your package
        'config',
        'dynamixel_hardware.yaml'
    ])
    # Load the dynamixel_controllers.yaml file
    dynamixel_controllers_config = PathJoinSubstitution([
        FindPackageShare('dynamixel_control'),
        'config',
        'dynamixel_controllers.yaml'
    ])


    return LaunchDescription([
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     parameters=[dynamixel_controllers_config],
        #     output='screen'
        # ),
        Node(
            package='dynamixel_control',
            executable='dynamixel_node',
            name='dynamixel_node',
            output='screen',
            parameters=[hardware_config_file]
        ),
        Node(
            package='rqt_joint_trajectory_controller',
            executable='rqt_joint_trajectory_controller',
            name='rqt_joint_trajectory_controller',
            output='screen',
        )
    ])