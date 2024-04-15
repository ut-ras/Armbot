#!/usr/bin/env python3

import os
import time

# Import ROS stuff
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory


# Uses Dynamixel SDK library and constants
# able to be found due to colcon build and resourcing of setup files.
# if this errors after doing above, go into directory of dynamixel_sdk and run pip install .
from dynamixel_sdk import *  # Uses Dynamixel SDK library

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
LED_ENABLE = 1
LED_DISABLE = 0


DXL_MOVING_STATUS_THRESHOLD = 20

ADDR_DRIVE_MODE = 10
ADDR_OPERATING_MODE = 11

ADDR_MX_TORQUE_ENABLE = 64
ADDR_MX_LED = 65
ADDR_MX_CW_ANGLE_LIMIT = 52
ADDR_MX_CCW_ANGLE_LIMIT = 48
# ADDR_MX_TORQUE_LIMIT = 34 #NOT USED, NO LOAD LIMIT SETTABLE IN 2XL430-W250-T
ADDR_MX_VEL_LIMIT = 44

ADDR_MX_GOAL_POSITION = 116

ADDR_MX_PRESENT_LOAD = 126
ADDR_MX_PRESENT_POSITION = 132

ADDR_MX_MOVING_SPEED = 112 # Profile Velocity
ADDR_MX_PRESENT_SPEED = 128

ADDR_MX_MOVING_STATUS = 123

class Dynamixel(Node):

    def __init__(self):
        super().__init__('dynamixel_node')
                # Load parameters from YAML file
        self.declare_parameter('dynamixel_hardware.ros__parameters.joint_name_1', 'DynaRotor_To_Dynamixel_J3')
        self.declare_parameter('dynamixel_hardware.ros__parameters.joint_name_2', 'DynaRotor_To_Dynamixel_J2')
        self.declare_parameter('dynamixel_hardware.ros__parameters.dynamixel_id_1', 1)
        self.declare_parameter('dynamixel_hardware.ros__parameters.dynamixel_id_2', 2)


        self.declare_parameter('dynamixel_hardware.ros__parameters.startup', True)
        self.declare_parameter('dynamixel_hardware.ros__parameters.device_name', '/dev/ttyUSB0')
        self.declare_parameter('dynamixel_hardware.ros__parameters.baudrate', 57600)
        self.declare_parameter('dynamixel_hardware.ros__parameters.protocol_version', 2.0)
        self.declare_parameter('dynamixel_a.ros__parameters.id', 1)
        # declare joints from the joint_names list respective to the order of the dynamixel_ids , 
        self.declare_parameter('dynamixel_a.ros__parameters.joint_name', 'DynaRotor_To_Dynamixel_J3')
        self.declare_parameter('dynamixel_a.ros__parameters.min_pos_limit_deg', 0)
        self.declare_parameter('dynamixel_a.ros__parameters.max_pos_limit_deg', 360.0)
        self.declare_parameter('dynamixel_a.ros__parameters.velocity_limit_rpm', 20.0)
        self.declare_parameter('dynamixel_a.ros__parameters.operating_mode', 4)
        self.declare_parameter('dynamixel_a.ros__parameters.driving_mode', 0)
        self.declare_parameter('dynamixel_b.ros__parameters.id', 2)
        # declare joints from the joint_names list respective to the order of the dynamixel_ids ,
        self.declare_parameter('dynamixel_b.ros__parameters.joint_name', 'DynaRotor_To_Dynamixel_J2')
        self.declare_parameter('dynamixel_b.ros__parameters.min_pos_limit_deg', 0)
        self.declare_parameter('dynamixel_b.ros__parameters.max_pos_limit_deg', 270)
        self.declare_parameter('dynamixel_b.ros__parameters.velocity_limit_rpm', 20.0)
        self.declare_parameter('dynamixel_b.ros__parameters.operating_mode', 3)
        self.declare_parameter('dynamixel_b.ros__parameters.driving_mode', 0)
        

        self.device_name = self.get_parameter('dynamixel_hardware.ros__parameters.device_name').value
        self.baudrate = self.get_parameter('dynamixel_hardware.ros__parameters.baudrate').value
        self.protocol_version = self.get_parameter('dynamixel_hardware.ros__parameters.protocol_version').value
        joint_name_1 = self.get_parameter('dynamixel_hardware.ros__parameters.joint_name_1').value
        joint_name_2 = self.get_parameter('dynamixel_hardware.ros__parameters.joint_name_2').value
        dynamixel_id_1 = self.get_parameter('dynamixel_hardware.ros__parameters.dynamixel_id_1').value
        dynamixel_id_2 = self.get_parameter('dynamixel_hardware.ros__parameters.dynamixel_id_2').value
        self.joint_names = [joint_name_1, joint_name_2]
        self.dynamixel_ids = [dynamixel_id_1, dynamixel_id_2]

        # Dynamixel A (J3)
        self.id_1 = self.get_parameter('dynamixel_a.ros__parameters.id').value
        self.joint_name_1 = self.get_parameter('dynamixel_a.ros__parameters.joint_name').value
        self.min_pos_limit_1 = self.get_parameter('dynamixel_a.ros__parameters.min_pos_limit_deg').value
        self.max_pos_limit_1 = self.get_parameter('dynamixel_a.ros__parameters.max_pos_limit_deg').value
        self.velocity_limit_1 = self.get_parameter('dynamixel_a.ros__parameters.velocity_limit_rpm').value
        self.operating_mode_1 = self.get_parameter('dynamixel_a.ros__parameters.operating_mode').value
        self.driving_mode_1 = self.get_parameter('dynamixel_a.ros__parameters.driving_mode').value

        # Dynamixel B (J2)
        self.id_2 = self.get_parameter('dynamixel_b.ros__parameters.id').value

        self.joint_name_2 = self.get_parameter('dynamixel_b.ros__parameters.joint_name').value
        self.min_pos_limit_2 = self.get_parameter('dynamixel_b.ros__parameters.min_pos_limit_deg').value
        self.max_pos_limit_2 = self.get_parameter('dynamixel_b.ros__parameters.max_pos_limit_deg').value
        self.velocity_limit_2 = self.get_parameter('dynamixel_b.ros__parameters.velocity_limit_rpm').value
        self.operating_mode_2 = self.get_parameter('dynamixel_b.ros__parameters.operating_mode').value
        self.driving_mode_2 = self.get_parameter('dynamixel_b.ros__parameters.driving_mode').value



        startup = self.get_parameter('dynamixel_hardware.ros__parameters.startup').value
        if startup:
            self.setupDynamixel()
        else:
            pass

        self.last_diff = 0

        self.servo_load_state = 0
        self.servo_load_state_ros = 0
        self.servo_speed_state = 0
        self.servo_speed_state_ros = 0
        self.servo_position_state = 0
        self.servo_position_state_ros = 0

        # # ROS connections
        # self.servo_load_state_pub = self.create_publisher(Int64, '~/servo_load_state', 10)
        # self.servo_speed_state_pub = self.create_publisher(Int64, '~/servo_speed_state', 10)
        # self.servo_position_state_pub = self.create_publisher(Int64, '~/servo_position_state', 10)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        # self.joint_trajectory_subscriber = self.create_subscription(
        #     JointTrajectory,
        #     '/fourDOF_ARM_Chain_controller/follow_joint_trajectory', # Update the topic name here
        #     self.joint_trajectory_callback,
        #     10
        # )
        self.joint_trajectory_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/fourDOF_ARM_Chain_controller/follow_joint_trajectory',
            self.execute_joint_trajectory_callback
        )
        
    def __del__(self):
        self.cleanup_all()


    def setupDynamixel(self):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.device_name)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.protocol_version)

        # Open port
        if self.portHandler.openPort():
            self.get_logger().info("Succeeded to open the port")
        else:
            self.get_logger().info("Failed to open the port")
            self.get_logger().info("Press any key to terminate...")
            
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.baudrate):
            self.get_logger().info("Succeeded to change the baudrate")
        else:
            self.get_logger().info("Failed to change the baudrate")
            self.get_logger().info("Press any key to terminate...")
            
            quit()

        # Enable Dynamixel Torque
        checkA1 = self.changeTorqueEnable(TORQUE_ENABLE, self.get_parameter('dynamixel_a.ros__parameters.id').value)
        if checkA1:
            self.get_logger().info("Dynamixel A has been successfully connected, setting up")
            
            self.get_logger().info("Setting Drive Mode for Dynamixel A")
            minPosLimit_1 = self.angle_to_decimal_deg(self.min_pos_limit_1)
            maxPosLimit_1 = self.angle_to_decimal_deg(self.max_pos_limit_2)
            self.get_logger().info('Min Pos Limit 1 is %r' % minPosLimit_1)
            self.get_logger().info('Max Pos Limit 1 is %r' % maxPosLimit_1)
            checkA2, checkA3, checkA4 = self.setOperatingMode_and_Limits(minPosLimit_1, maxPosLimit_1, self.id_1, self.operating_mode_1, 0)
            #  error handling
            if not checkA2:
                    self.get_logger().info("Failed to set Operating Mode for Dynamixel A")
                    self.get_logger().info("Press any key to terminate...")
                    quit()
            if not checkA3:
                    self.get_logger().info("Failed to set CW Angle Limit for Dynamixel A")
                    self.get_logger().info("Press any key to terminate...")
                    quit()
            if not checkA4:
                    self.get_logger().info("Failed to set CCW Angle Limit for Dynamixel A")
                    self.get_logger().info("Press any key to terminate...")
                    quit()
                


        checkB1 = self.changeTorqueEnable(TORQUE_ENABLE, self.get_parameter('dynamixel_b.ros__parameters.id').value)

        if checkB1:
            self.get_logger().info("Dynamixel B has been successfully connected")
            self.get_logger().info("Setting Drive Mode for Dynamixel B")
            minPosLimit_2 = self.angle_to_decimal_deg(self.min_pos_limit_2)
            maxPosLimit_2 = self.angle_to_decimal_deg(self.max_pos_limit_2)
            self.get_logger().info('Min Pos Limit 2 is %r' % minPosLimit_2)
            self.get_logger().info('Max Pos Limit 2 is %r' % maxPosLimit_2)
            checkB2, checkB3, checkB4 = self.setOperatingMode_and_Limits(minPosLimit_2, maxPosLimit_2, self.id_2, self.operating_mode_2, 1)
      
            if not checkB2:
                self.get_logger().info("Failed to set Operating Mode for Dynamixel B")
                self.get_logger().info("Press any key to terminate...")
                
                quit()

            if not checkB3:
                self.get_logger().info("Failed to set CW Angle Limit for Dynamixel B")
                self.get_logger().info("Press any key to terminate...")
                
                quit()

            if not checkB4:
                self.get_logger().info("Failed to set CCW Angle Limit for Dynamixel B")
                self.get_logger().info("Press any key to terminate...")
                
                quit()

        # Enable all Torques
        for ids in self.dynamixel_ids:
            self.changeTorqueEnable(TORQUE_ENABLE, ids)
        #Done setting up Dynamixel Motors

        self.get_logger().info("Dynamixel Motors have been successfully set up!")
        self.get_logger().info("Moving to zero position")
        self.setPosition(0, self.id_1)
        self.setPosition(0, self.id_2)
        # self.publish_joint_states()

    def execute_joint_trajectory_callback(self, goal_handle):
        # Extract joint trajectory from the goal
        joint_trajectory = goal_handle.request.trajectory

        # Execute the trajectory
        result = self.execute_joint_trajectory(joint_trajectory)

        # Send the result back to the action client (MoveIt!)
        goal_handle.succeed()
        return result

    def execute_joint_trajectory(self, joint_trajectory):
        result = FollowJointTrajectory.Result()
        # Iterate through the trajectory points and execute them
        for point in joint_trajectory.points:
            # Extract joint positions from the trajectory point
            for i, joint_name in enumerate(joint_trajectory.joint_names):
                if joint_name in self.joint_names:
                    dynamixel_id = self.dynamixel_ids[self.joint_names.index(joint_name)]
                    position = point.positions[i]

                    # Send commands to the Dynamixel motor
                    self.get_logger().info('Setting position of Dynamixel ID %d to %f' % (dynamixel_id, self.angle_to_decimal(position)))
                    self.setPosition(self.angle_to_decimal(position), dynamixel_id)

        return result


    def publish_joint_states(self):
        # Retrieve current positions, velocities, and efforts from the Dynamixel motors
        positions = []
        velocities = []
        # efforts = []
        for dynamixel_id in self.dynamixel_ids:
            positions.append(self.decimal_to_angle(self.getPosition(dynamixel_id)))
            velocities.append(self.decimal_to_rpm(self.getSpeed(dynamixel_id)))
            # efforts.append(0.0)  # Placeholder for effort 

        # Create and populate the JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = positions
        joint_state_msg.velocity = velocities
        # joint_state_msg.effort = efforts

        # Publish the joint states
        self.joint_state_publisher.publish(joint_state_msg)
    def changeTorqueEnable(self, value, id):

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_ENABLE, value)
        if value == TORQUE_ENABLE:
            self.get_logger().info('Torque of Dynamixel ID %d has been enabled' % id)
        else:
            self.get_logger().info('Torque of Dynamixel ID %d has been disabled' % id)
        # set LED to indicate torque status
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_MX_LED, value)
        return self.checkResults(dxl_comm_result, dxl_error)

    def setOperatingMode_and_Limits(self, minPosLimit, maxPosLimit, id, DesiredOperatingMode, ReverseDriveMode):
        # Disable ALL Torques on all servos
        for ids in self.dynamixel_ids:
            self.changeTorqueEnable(TORQUE_DISABLE, ids)

        # Set Operating Mode
        """ 
        Operating Mode:
        1: Velocity Control Mode (Continuous Rotation / Wheel Mode)
        3: Position Control Mode (default) (minPosLimit ~ maxPosLimit)
        4: Extended Position Control Mode (Multi-turn) (-512 rev ~ +512 rev)
        16: PWM Control Mode 
        # """
        dxl_comm_result1, dxl_error1 = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_OPERATING_MODE, DesiredOperatingMode)
        
        count = 0
        while True:
            count = count + 1
            check1 = self.checkResults(dxl_comm_result1, dxl_error1)
            self.get_logger().info('Check1 is %r' % check1)
            if check1:
                break
            if count > 10:
                self.get_logger().info('Max Count Reached!')
                count = 0
                break
        dxl_comm_result1, dxl_error1 = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_DRIVE_MODE, ReverseDriveMode)
        count = 0
        while True:
            count = count + 1
            check1 = self.checkResults(dxl_comm_result1, dxl_error1)
            self.get_logger().info('Check1 is %r' % check1)
            if check1:
                break
            if count > 10:
                self.get_logger().info('Max Count Reached!')
                count = 0
                break

        # Change CW and CCW angle limit
        dxl_comm_result2, dxl_error2 = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_MX_CW_ANGLE_LIMIT, minPosLimit)
        dxl_comm_result3, dxl_error3 = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_MX_CCW_ANGLE_LIMIT, maxPosLimit)
        while True:
            count = count + 1
            check2 = self.checkResults(dxl_comm_result2, dxl_error2)
            check3 = self.checkResults(dxl_comm_result3, dxl_error3)
            self.get_logger().info('Check2 is %r' % check2)
            self.get_logger().info('Check3 is %r' % check3)
            if check2 and check3:
                break
            if count > 10:
                self.get_logger().info('Max Count Reached!')
                count = 0
                break
            dxl_comm_result2, dxl_error2 = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_MX_CW_ANGLE_LIMIT, minPosLimit)
            dxl_comm_result3, dxl_error3 = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_MX_CCW_ANGLE_LIMIT, maxPosLimit)
        self.changeTorqueEnable(TORQUE_ENABLE, id)
        return check1, check2, check3

    # def setTorqueLimit(self, percent, id): #NOT USED, NO LOAD LIMIT SETTABLE IN 2XL430-W250-T
    #     mod = float(percent / 100.0)
    #     val = int(1023 * mod)
    #     dxl_comm_result, dxl_error = self.packetHandler.write4ByteTXRX(self.portHandler, id, ADDR_MX_TORQUE_LIMIT, val)
    #     check = self.checkResults(dxl_comm_result, dxl_error)
    #     if check:
    #         self.get_logger().info('Torque Limit set to %d.', val)
    #     else:
    #         self.get_logger().info('Something went wrong.')

    def setVelocityLimit(self, percent, id):
        self.changeTorqueEnable(TORQUE_DISABLE, id)
        mod = float(percent / 100.0)
        val = int(1023 * mod)  # Initial Value shown in EEPROM is 230, resolution is 0.229 rpm
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTXRX(self.portHandler, id, ADDR_MX_VEL_LIMIT, val)
        check = self.checkResults(dxl_comm_result, dxl_error)
        if check:
            self.get_logger().info('Velocity Limit set to %d.', val)
        else:
            self.get_logger().info('Something went wrong.')
        self.changeTorqueEnable(TORQUE_ENABLE, id)

    def getLoad(self, id):
        # Read present load
        dxl_present_load, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_LOAD)
        while True:
            check = self.checkResults(dxl_comm_result, dxl_error)
            if check:
                break
            dxl_present_load, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_LOAD)
        # self.servo_load_state = dxl_present_load
        # self.servo_load_state_ros.data = self.servo_load_state
        # self.servo_load_state_pub.publish(self.servo_load_state_ros)
        return dxl_present_load


    # Multi-Turn and Wheel Related
    def getServoStates(self, which, id):
        if which == "load":
            # get and publish servo load state
            self.getLoad(id)
        elif which == "speed":
            # get and publish servo speed state
            self.getSpeed(id)
        elif which == "both":
            self.getLoad(id)
            time.sleep(0.05)
            self.getSpeed(id)

    # Multi-Turn Related

    def getPosition(self, id):
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_POSITION)
        while True:
            check = self.checkResults(dxl_comm_result, dxl_error)
            if check:
                break
            time.sleep(0.05)
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_POSITION)
        # self.servo_position_state = dxl_present_position
        # self.servo_position_state_ros.data = dxl_present_position
        # self.servo_position_state_pub.publish(self.servo_position_state_ros)
        return dxl_present_position

    def setPosition(self, position_cmd, id):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION, position_cmd)
        count = 0
        while True: 
            check = self.checkResults(dxl_comm_result, dxl_error)
            if check:
                break
            time.sleep(0.3)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION, position_cmd)
            count = count + 1
            if count > 3:
                self.get_logger().info('Max Count Reached!')
                count = 0
                break

        last_diff = 0
        # while True:
        #     # Read present position
        #     dxl_present_position = self.getPosition(id)
        #     time.sleep(0.3)
        #     self.get_logger().info("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (id, position_cmd, dxl_present_position))
        #     diff = position_cmd - dxl_present_position
        #     if diff > 0 > last_diff:
        #         self.get_logger().info('CW Position reached')
        #         break
        #     elif diff < 0 < last_diff:
        #         self.get_logger().info('CCW Position reached')
        #         break
        #     elif abs(position_cmd - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD:
        #         self.get_logger().info('A Position is reached')
        #         break
        #     last_diff = diff

    def setPosition_withLoadCheck(self, position_cmd, max_load, id):
        count = 0
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTXRX(self.portHandler, id, ADDR_MX_GOAL_POSITION, position_cmd)
        while True:
            check = self.checkResults(dxl_comm_result, dxl_error)
            self.get_logger().info('Check is %r' % check)
            if check:
                break
            time.sleep(0.3)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTXRX(self.portHandler, id, ADDR_MX_GOAL_POSITION, position_cmd)
        last_diff = 0
        while True:
            count = count + 1
            if count > 5 and count % 2 == 0:
                self.get_logger().info('Checking for Load')
                dxl_present_load = self.getLoad(id)
                if dxl_present_load >= max_load:
                    self.get_logger().info('Load Limit has been exceeded')
                    return False
            # Read present position
            dxl_present_position = self.getPosition(id)
            time.sleep(0.3)
            self.get_logger().info("[ID:%03d] GoalPos:%03d  PresPos:%03d", id, position_cmd, dxl_present_position)
            diff = position_cmd - dxl_present_position
            if diff > 0 > last_diff:
                self.get_logger().info('CW Position reached')
                return True
            elif diff < 0 < last_diff:
                self.get_logger().info('CCW Position reached')
                return True
            elif abs(position_cmd - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD:
                self.get_logger().info('A Position is reached')
                return True
            last_diff = diff

    def setPosition_withPositionCheck(self, position_cmd, id):
        count = 0
        pos_same_count = 0
        last_pos = 0
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTXRX(self.portHandler, id, ADDR_MX_GOAL_POSITION, position_cmd)
        while True:
            check = self.checkResults(dxl_comm_result, dxl_error)
            self.get_logger().info('Check is %r' % check)
            if check:
                break
            time.sleep(0.3)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTXRX(self.portHandler, id, ADDR_MX_GOAL_POSITION, position_cmd)
        last_diff = 0
        while True:
            dxl_present_position = self.getPosition(id)
            count = count + 1
            if count > 5 and count % 3 == 0:
                self.get_logger().info('Checking for Position')
                if dxl_present_position == last_pos:
                    pos_same_count += 1
                    self.get_logger().info('Position Same as Last Time')
                if pos_same_count >= 5:
                    self.get_logger().info('5 Position same counts reached')
                    return False
                last_pos = dxl_present_position
            time.sleep(0.3)
            diff = position_cmd - dxl_present_position
            if diff > 0 > last_diff:
                self.get_logger().info('CW Position reached')
                return True
            elif diff < 0 < last_diff:
                self.get_logger().info('CCW Position reached')
                return True
            elif abs(position_cmd - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD:
                self.get_logger().info('A Position is reached')
                return True
            last_diff = diff

    def checkResults(self, result, error):
        if result != COMM_SUCCESS:
            self.get_logger().info(f"TxRxResult: {self.packetHandler.getTxRxResult(result)}")
            return False
        elif error != 0:
            self.get_logger().info(f"RxPacketError: {self.packetHandler.getRxPacketError(error)}")
            return False
        else:
            return True

    # Speed Related

    def getSpeed(self, id):
        # Read present speed
        dxl_present_speed, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_SPEED)
        while True:
            check = self.checkResults(dxl_comm_result, dxl_error)
            if check:
                break
            dxl_present_speed, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_SPEED)
        # self.servo_speed_state = dxl_present_speed
        # self.servo_speed_state_ros.data = self.servo_speed_state
        # self.servo_speed_state_pub.publish(self.servo_speed_state_ros)
        return dxl_present_speed
    def setSpeed(self, speed, id):
       # Write new moving speed
       dxl_comm_result, dxl_error = self.packetHandler.write4ByteTXRX(self.portHandler, id, ADDR_MX_MOVING_SPEED, speed)
       while True:
           check = self.checkResults(dxl_comm_result, dxl_error)
           if check:
               break
           dxl_comm_result, dxl_error = self.packetHandler.write4ByteTXRX(self.portHandler, id, ADDR_MX_MOVING_SPEED, speed)
       return check

   # Shutdown Related

    def stopServo(self, speed, id):
        inner_step = 0
        while True:
            if inner_step < 1000:
                pass
            else:
                self.setSpeed(-1 * speed, id)
                break
            inner_step += 1

    def disableDynamixel(self, id):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        check = self.checkResults(dxl_comm_result, dxl_error)
        if check:
            self.get_logger().info("Dynamixel Torque has been successfully disabled")
        # Close port
        self.portHandler.closePort()
        self.get_logger().info("Port for Dynamixel has been commanded to be closed")

    def cleanup(self, id):
        # Disable Dynamixel Torque and close port
        self.disableDynamixel(id)

    def cleanup_all(self):
        for ids in self.dynamixel_ids:
            self.cleanup(ids)
    def decimal_to_angle(self, decimal):
        decimal *= 0.088 # 0.088 degrees per unit (0 ~ 4095)
        decimal *= 0.0174533 # Convert to radians
        return decimal # radians

    def angle_to_decimal_deg(self, angle):
        return int(angle / 0.088)
    
    def angle_to_decimal(self, angle): #used for converting ros angle to dynamixel angle

        # rad to deg, then if negative find corrected angle in deg,  then / 0.088
        angle = angle * 57.2958
        return int(angle / 0.088)
    def decimal_to_rpm(self, decimal):
        return decimal * 0.229 # 0.229 rpm per unit (0 ~ 1023)
    def rpm_to_decimal(self, rpm):
        return int(rpm / 0.229)
    def decimal_to_load(self, decimal):
        return decimal * 0.1 # 0.1% per unit (-1000 ~ 1000)
    def load_to_decimal(self, load):
        return int(load / 0.1)
    

def main(args=None):
    rclpy.init(args=args)
    dynamixel = Dynamixel()

    # Create a timer to periodically publish joint states
    joint_state_timer = dynamixel.create_timer(0.1, dynamixel.publish_joint_states)


    try:
        rclpy.spin(dynamixel)  # Blocks execution until node is shutdown
    finally:
        # Ensure that the node and resources are properly cleaned up
        # dynamixel.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()