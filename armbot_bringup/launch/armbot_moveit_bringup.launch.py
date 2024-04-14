from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("armbot", package_name="armbot_moveit").to_moveit_configs()
    return generate_demo_launch(moveit_config)

