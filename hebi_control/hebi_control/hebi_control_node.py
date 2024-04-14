import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import hebi

class HebiControlNode(Node):
    def __init__(self):
        super().__init__('hebi_control_node')
        # Initialize Hebi lookup and group
        self.lookup = hebi.Lookup()
        self.group = self.lookup.get_group_from_names(['family'], ['motor1', 'motor2'])
        
        # Create publishers and subscribers
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_trajectory_subscriber = self.create_subscription(JointTrajectory, 'joint_trajectory', self.joint_trajectory_callback, 10)
        
        # Initialize feedback and command
        self.feedback = hebi.GroupFeedback(self.group.size)
        self.command = hebi.GroupCommand(self.group.size)

    def joint_trajectory_callback(self, msg):
        # Convert JointTrajectory message to Hebi command
        # Update self.command with the desired positions, velocities, and accelerations
        # Send the command to the Hebi group
        self.group.send_command(self.command)

    def publish_joint_states(self):
        # Get feedback from the Hebi group
        self.group.get_next_feedback(reuse_fbk=self.feedback)
        
        # Create a JointState message and populate it with the current positions and velocities
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint1', 'joint2']  # Names of your Hebi joints
        joint_state_msg.position = self.feedback.position
        joint_state_msg.velocity = self.feedback.velocity
        
        # Publish the joint states
        self.joint_state_publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    hebi_control_node = HebiControlNode()
    rclpy.spin(hebi_control_node)
    hebi_control_node.destroy_node()
    rclpy.shutdown()

    