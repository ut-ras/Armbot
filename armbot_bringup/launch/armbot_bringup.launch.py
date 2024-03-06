from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'armbot_bringup'

    pkg_dir = get_package_share_directory(pkg_name)

    # Define the path to the URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'armbot.urdf.xacro')

    # Use xacro to generate the URDF
    robot_description_config = ExecuteProcess(
        cmd=['xacro', urdf_file],
        output='screen',
        shell=True
    )

    # Node to publish the robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_config}],
    )

    # Optional: Launch the joint_state_publisher_gui for manual joint control
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'armbot.rviz')],
        output='screen'
    )

    return LaunchDescription([
        robot_description_config,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
