#!/usr/bin/env python3
import rclpy
import math
from sensor_msgs.msg import JointState
from elite_msgs.msg import RobotState
from std_msgs.msg import Header

class EliteStatePublisher():
    def __init__(self):
        self.get_logger().info("starting elite state publishers")
        # Publish joint states
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)
        # Todo Ros parameter
        self.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Publish robot states
        self.robot_state_publisher_ = self.create_publisher(
            RobotState, 'robot_state', 10)
        self.robot_state = RobotState()
        self.robot_state.header = Header()

    def update_states(self):
        self.update_joint_state()
        self.update_robot_state()

    def update_joint_state(self):
        self.joint_state = JointState()
        self.joint_state.name = self.joint_name

        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        for joint in self.elite_robot.monitor_info.machinePos[:6]:  # pylint: disable=E1101
            self.joint_state.position.append(math.radians(joint))
        
        for speed in self.elite_robot.monitor_info.joint_speed[:6]:  # pylint: disable=E1101
            self.joint_state.velocity.append(math.radians(speed))
        self.joint_state.effort = self.elite_robot.monitor_info.torque[
            :6]

        self.joint_state_publisher.publish(self.joint_state)

    def update_robot_state(self):
        self.robot_state.header.stamp = self.get_clock().now().to_msg()
        
        self.robot_state.analog_io_input = self.elite_robot.monitor_info.analog_ioInput
        self.robot_state.analog_io_output = self.elite_robot.monitor_info.analog_ioOutput
        self.robot_state.autorun_cycle_mode = self.elite_robot.monitor_info.autorun_cycleMode

        self.robot_state.can_motor_run = self.elite_robot.monitor_info.can_motor_run
        self.robot_state.collision = self.elite_robot.monitor_info.collision

        self.robot_state.digital_io_input = self.elite_robot.monitor_info.digital_ioInput
        self.robot_state.digital_io_output = self.elite_robot.monitor_info.digital_ioOutput

        self.robot_state.emergency_stop_state = self.elite_robot.monitor_info.emergencyStopState

        self.robot_state.joint_speed = self.elite_robot.monitor_info.joint_speed
        self.robot_state.joint_acc = self.elite_robot.monitor_info.jointacc

        self.robot_state.motor_speed = self.elite_robot.monitor_info.motor_speed
        self.robot_state.machine_user_flange_pose = self.elite_robot.monitor_info.machineUserFlangePose
        self.robot_state.machine_user_pose = self.elite_robot.monitor_info.machineUserPose
        self.robot_state.machine_pos = self.elite_robot.monitor_info.machinePos
        self.robot_state.machine_pose = self.elite_robot.monitor_info.machinePose
        self.robot_state.machine_flange_pose = self.elite_robot.monitor_info.machineFlangePose

        self.robot_state.robot_mode = self.elite_robot.monitor_info.robotMode
        self.robot_state.robot_state = self.elite_robot.monitor_info.robotState

        self.robot_state.servo_ready = self.elite_robot.monitor_info.servoReady

        self.robot_state.tcp_speed = self.elite_robot.monitor_info.tcp_speed
        self.robot_state.torque = self.elite_robot.monitor_info.torque
        self.robot_state.tcpacc = self.elite_robot.monitor_info.tcpacc
        self.robot_state_publisher_.publish(self.robot_state)
