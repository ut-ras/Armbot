from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from srdfdom.srdf import SRDF
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

def generate_launch_description():
    # Define the path to the dynamixel_controllers.yaml file within your package
    dynamixel_hardware_config_file = PathJoinSubstitution([
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

    moveit_config = MoveItConfigsBuilder("armbot", package_name="armbot_moveit").to_moveit_configs()

    return generate_spawn_controllers_launch_custom(moveit_config, dynamixel_hardware_config_file, dynamixel_controllers_config)

def generate_spawn_controllers_launch_custom(moveit_config, dynamixel_hardware_config_file, dynamixel_controllers_config):
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])

    ld = LaunchDescription()

    # Add the controller manager
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[dynamixel_controllers_config],
            output="screen",
        ),
    )


    for controller in controller_names + ["joint_state_broadcaster"]:
        ld.add_action(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[controller],
                    output="screen",
                ),
            )

    # Add the dynamixel_node
    ld.add_action(
        Node(
            package='dynamixel_control',
            executable='dynamixel_node',
            name='dynamixel_node',
            output='screen',
            parameters=[dynamixel_hardware_config_file]
        )
    ),
    ld.add_action(
        Node(
            package='rqt_joint_trajectory_controller',
            executable='rqt_joint_trajectory_controller',
            name='rqt_joint_trajectory_controller',
            output='screen',
        )
    )
    return ld