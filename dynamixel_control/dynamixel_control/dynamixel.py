#!/usr/bin/env python3

import os
import time

# Import ROS stuff
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# Uses Dynamixel SDK library and constants
# able to be found due to colcon build and resourcing of setup files.
# if this errors after doing above, go into directory of dynamixel_sdk and run pip install .
from dynamixel_sdk import *  # Uses Dynamixel SDK library


class Dynamixel(Node):
    def __init__(self, startup):
        super().__init__('dynamixel_node')
        if startup:
            self.setupDynamixel()
        else:
            pass

        self.last_diff = 0

        self.servo_load_state = 0
        self.servo_load_state_ros = Int32()
        self.servo_speed_state = 0
        self.servo_speed_state_ros = Int32()
        self.servo_position_state = 0
        self.servo_position_state_ros = Int32()

        # ROS connections
        self.servo_load_state_pub = self.create_publisher(Int32, '~/servo_load_state', 10)
        self.servo_speed_state_pub = self.create_publisher(Int32, '~/servo_speed_state', 10)
        self.servo_position_state_pub = self.create_publisher(Int32, '~/servo_position_state', 10)

    # Torque/Startup-Related

    def setupDynamixel(self):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            self.get_logger().info("Succeeded to open the port")
        else:
            self.get_logger().info("Failed to open the port")
            self.get_logger().info("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info("Succeeded to change the baudrate")
        else:
            self.get_logger().info("Failed to change the baudrate")
            self.get_logger().info("Press any key to terminate...")
            getch()
            quit()

        # Enable Dynamixel Torque
        check1 = self.changeTorqueEnable(TORQUE_ENABLE, 1)
        if check1:
            self.get_logger().info("Dynamixel 1 has been successfully connected")

        check2 = self.changeTorqueEnable(TORQUE_ENABLE, 2)
        if check2:
            self.get_logger().info("Dynamixel 2 has been successfully connected")

    def changeTorqueEnable(self, value, id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_ENABLE, value)
        return self.checkResults(dxl_comm_result, dxl_error)

    def setOperatingMode(self, reg_six_value, reg_eight_value, id):
        self.changeTorqueEnable(TORQUE_DISABLE, id)
        # Change CW and CCW angle limit
        dxl_comm_result1, dxl_error1 = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_CW_ANGLE_LIMIT, reg_six_value)
        dxl_comm_result2, dxl_error2 = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_CCW_ANGLE_LIMIT, reg_eight_value)
        while True:
            check1 = self.checkResults(dxl_comm_result1, dxl_error1)
            check2 = self.checkResults(dxl_comm_result2, dxl_error2)
            self.get_logger().info('Check1 is %r', check1)
            self.get_logger().info('Check2 is %r', check2)
            if check1 and check2:
                break
            dxl_comm_result1, dxl_error1 = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_CW_ANGLE_LIMIT, reg_six_value)
            dxl_comm_result2, dxl_error2 = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_CCW_ANGLE_LIMIT, reg_eight_value)
        self.changeTorqueEnable(TORQUE_ENABLE, id)
        return check1, check2

    def setTorqueLimit(self, percent, id):
        mod = float(percent / 100.0)
        val = int(1023 * mod)
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_LIMIT, val)
        check = self.checkResults(dxl_comm_result, dxl_error)
        if check:
            self.get_logger().info('Torque Limit set to %d.', val)
        else:
            self.get_logger().info('Something went wrong.')

    def setVelocityLimit(self, percent, id):
        self.changeTorqueEnable(TORQUE_DISABLE, id)
        mod = float(percent / 100.0)
        val = int(1023 * mod)  # Initial Value shown in EEPROM is 230, resolution is 0.229 rpm
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_VEL_LIMIT, val)
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
        self.servo_load_state = dxl_present_load
        self.servo_load_state_ros.data = self.servo_load_state
        self.servo_load_state_pub.publish(self.servo_load_state_ros)
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
            self.sleep_for(0.05)
            self.getSpeed(id)

    # Multi-Turn Related

    def getPosition(self, id):
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_POSITION)
        while True:
            check = self.checkResults(dxl_comm_result, dxl_error)
            if check:
                break
            self.sleep_for(0.05)
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_POSITION)
        self.servo_position_state = dxl_present_position
        self.servo_position_state_ros.data = dxl_present_position
        self.servo_position_state_pub.publish(self.servo_position_state_ros)
        return dxl_present_position

    def setPosition(self, position_cmd, id):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION, position_cmd)
        while True:
            check = self.checkResults(dxl_comm_result, dxl_error)
            if check:
                break
            self.sleep_for(0.3)
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION, position_cmd)
        last_diff = 0
        while True:
            # Read present position
            dxl_present_position = self.getPosition(id)
            self.sleep_for(0.3)
            self.get_logger().info("[ID:%03d] GoalPos:%03d  PresPos:%03d", id, position_cmd, dxl_present_position)
            diff = position_cmd - dxl_present_position
            if diff > 0 > last_diff:
                self.get_logger().info('CW Position reached')
                break
            elif diff < 0 < last_diff:
                self.get_logger().info('CCW Position reached')
                break
            elif abs(position_cmd - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD:
                self.get_logger().info('A Position is reached')
                break
            last_diff = diff

    def setPosition_withLoadCheck(self, position_cmd, max_load, id):
        count = 0
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION, position_cmd)
        while True:
            check = self.checkResults(dxl_comm_result, dxl_error)
            self.get_logger().info('Check is %r', check)
            if check:
                break
            self.sleep_for(0.3)
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION, position_cmd)
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
            self.sleep_for(0.3)
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
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION, position_cmd)
        while True:
            check = self.checkResults(dxl_comm_result, dxl_error)
            self.get_logger().info('Check is %r', check)
            if check:
                break
            self.sleep_for(0.3)
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION, position_cmd)
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
            self.sleep_for(0.3)
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
            self.get_logger().info("%s", self.packetHandler.getTxRxResult(result))
            return False
        elif error != 0:
            self.get_logger().info("%s", self.packetHandler.getRxPacketError(error))
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
        self.servo_speed_state = dxl_present_speed
        self.servo_speed_state_ros.data = self.servo_speed_state
        self.servo_speed_state_pub.publish(self.servo_speed_state_ros)
        return dxl_present_speed
    def setSpeed(self, speed, id):
       # Write new moving speed
       dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_MOVING_SPEED, speed)
       while True:
           check = self.checkResults(dxl_comm_result, dxl_error)
           if check:
               break
           dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_MOVING_SPEED, speed)
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


def main(args=None):
    rclpy.init(args=args)
    dynamixel = Dynamixel(startup=True)
    rclpy.spin(dynamixel)
    dynamixel.cleanup(1)  # Assuming cleanup for Dynamixel ID 1
    dynamixel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()