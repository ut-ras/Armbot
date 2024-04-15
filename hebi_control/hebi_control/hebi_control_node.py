import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import hebi
from time import sleep
import numpy as np


class HebiControlNode(Node):
    def __init__(self):
        super().__init__('hebi_control_node')

        self.get_logger().info('Initializing HebiControlNode')

        # Initialize Hebi lookup and group
        self.lookup = hebi.Lookup()
        # Give the Lookup process 2 seconds to discover modules
        sleep(2)
        print('Modules found on network:')
        for entry in self.lookup.entrylist:
            print(f'{entry.family} | {entry.name}')

        # Try to get the group using different methods
        self.group = None
        try:
            self.group = self.lookup.get_group_from_family('Armbot')
            self.get_logger().info('Got group from family')
        except Exception as e:
            self.get_logger().error('Failed to get group from names: {0}'.format(str(e)))

        if self.group is None:
            try:
                self.group = self.lookup.get_group_from_macs('D8:80:39:EF:01:7A')
                self.get_logger().info('Got group from MAC address')
            except Exception as e:
                self.get_logger().error('Failed to get group from MAC address: {0}'.format(str(e)))

        if self.group is None:
            raise RuntimeError('Failed to create Hebi group')
            quit()
        else:
            self.get_logger().info('Hebi group initialized')

        # # Get gains xml files from config/gains folder
        # self.groupinfo = self.group.request_info()
        # if self.groupinfo is None:
        #     raise RuntimeError('Failed to get group info')
        #     quit()
        # else:
        #     self.get_logger().info('Got group info')
        #     self.group_info.write_gains('config/gains/HEBI_GAINS.xml')
        self.command = hebi.GroupCommand(self.group.size)

        self.command = hebi.GroupCommand(self.group.size)
        self.command.read_gains("/home/parallels/armbot_ws/hebi_control/config/gains/HEBI_GAINS.xml")
        self.group.send_command_with_acknowledgement(self.command)

        # Create publishers and subscribers
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.get_logger().info('Publisher created')

        # Initialize feedback and command
        self.feedback = hebi.GroupFeedback(self.group.size)

        # Create action server for joint trajectory
        self.joint_trajectory_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/fourDOF_ARM_Chain_controller/follow_joint_trajectory',
            self.execute_joint_trajectory_callback
        )
        self.get_logger().info('Action server created')

        # Send to zero positions
        self.command.position = [float(0.0), float(0.0)]
        self.group.send_command(self.command)
        self.get_logger().info('Sent initial command')

    def execute_joint_trajectory_callback(self, goal_handle):
        self.get_logger().info('Received a goal')
        # Extract joint trajectory from the goal
        joint_trajectory = goal_handle.request.trajectory

        # Execute the trajectory
        result = self.execute_joint_trajectory(joint_trajectory)

        # Send the result back to the action client (MoveIt!)
        goal_handle.succeed()
        self.get_logger().info('Goal succeeded')
        return result

    def execute_joint_trajectory(self, joint_trajectory):
        result = FollowJointTrajectory.Result()
        
        # Buffer to store the trajectory points
        point_buffer = []
        
        # Iterate through the trajectory points and store them in the buffer
        for point in joint_trajectory.points:
            # Extract joint positions from the trajectory point
            positions = []
            for i, joint_name in enumerate(joint_trajectory.joint_names):
                if joint_name in ['Hebi_Rotor_Base', 'Hebi_Rotor_J1']:  # Replace with your Hebi joint names
                    position = point.positions[i]
                    self.get_logger().info('Received joint position: {0}'.format(position))
                    positions.append(position)
            
            if len(positions) == 2:  # Assuming you have 2 Hebi joints
                point_buffer.append(positions)
            else:
                self.get_logger().error('Received invalid joint positions')
        
        # Execute the buffered trajectory points with a small delay between each point
        for positions in point_buffer:
            # Send commands to the Hebi motors
            self.command.position = positions
            self.group.send_command(self.command)
            self.get_logger().info('Sent command to Hebi motors: {0}'.format(positions))
            
            # Add a small delay to allow the motors to reach the commanded position
            sleep(0.1)  # Adjust the delay as needed
        
        # Continuously send the last commanded position until a new trajectory is received
        while rclpy.ok():
            if point_buffer:
                last_positions = point_buffer[-1]
                self.command.position = last_positions
                self.group.send_command(self.command)
                self.get_logger().info('Holding last commanded position: {0}'.format(last_positions))
            
            # Add a small delay to avoid overwhelming the motors with commands
            sleep(0.01)  # Adjust the delay as needed
            
            # Check if a new trajectory has been received
            if self.joint_trajectory_action_server.is_preempt_requested():
                self.get_logger().info('New trajectory received, stopping hold position')
                break
        
        return result
    def publish_joint_states(self):
        # Get feedback from the Hebi group
        self.group.get_next_feedback(reuse_fbk=self.feedback)
        
        # Create a JointState message and populate it with the current positions and velocities
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['Hebi_Rotor_Base', 'Hebi_Rotor_J1']  # Replace with your Hebi joint names
        
        # Assign positions and velocities to the message fields
        joint_state_msg.position = self.feedback.position.tolist()
        joint_state_msg.velocity = self.feedback.velocity.tolist()
        
        # Publish the joint states
        self.joint_state_publisher.publish(joint_state_msg)
        # self.get_logger().info('Published joint states: {0}'.format(joint_state_msg))


def main(args=None):
    rclpy.init(args=args)
    hebi_control_node = HebiControlNode()

    # Create a timer to periodically publish joint states
    joint_state_timer = hebi_control_node.create_timer(0.1, hebi_control_node.publish_joint_states)

    rclpy.spin(hebi_control_node)
    hebi_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()