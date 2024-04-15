from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define the URDF package name
    urdf_pkg_name = 'armbot_description'

    # Find the URDF package directory
    urdf_pkg_dir = FindPackageShare(package=urdf_pkg_name).find(urdf_pkg_name)

    # Define the path to the URDF file within the 'armbot_description' package
    urdf_file_path = f"{urdf_pkg_dir}/onshape_to_robot/armbot/robot.urdf"

    # Use premade urdf file
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # Node to publish the robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    # Optional: Launch the joint_state_publisher_gui for manual joint control
    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui'
    # )

    # Define the package name for the launch configuration
    pkg_name = 'armbot_bringup'

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'rviz',
        'armbot.rviz'  # replace 'armbot.rviz' with 'specific_file.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        # joint_state_publisher_gui,
        # rviz
    ])