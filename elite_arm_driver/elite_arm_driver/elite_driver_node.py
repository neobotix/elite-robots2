#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import time

from elite import EC
from .elite_arm_kinematics import EliteArmKinematics
from .elite_arm_move import EliteArmMove
from .elite_arm_set_io import EliteArmSetIO
from .elite_state_publisher import EliteStatePublisher
from .elite_arm_fake_ec import EliteArmFakeEc
from .elite_arm_trajectory_action import EliteArmTrajectoryAction


class EliteDriver(Node, EliteArmKinematics, EliteArmMove, EliteArmSetIO, EliteStatePublisher, EliteArmTrajectoryAction):

    def __init__(self) -> None:
        super().__init__('elite_driver_node')
        EliteArmKinematics.__init__(self)
        EliteArmMove.__init__(self)
        EliteArmSetIO.__init__(self)
        EliteStatePublisher.__init__(self)
        EliteArmTrajectoryAction.__init__(self)

        # declare parameters
        self.declare_parameter('ip_address', value = "192.168.1.200")
        self.declare_parameter('auto_connect', value = True)
        self.declare_parameter('use_fake', value = False)
        self.declare_parameter('time_period', value=0.01)

        # Setup ros parameters
        self.ip_address = self.get_parameter('ip_address').value
        self.auto_connect = self.get_parameter('auto_connect').value
        self.use_fake = self.get_parameter('use_fake').value
        timer_period = self.get_parameter('time_period').value

        self.elite_robot = None
        self.create_timer(timer_period, self.publish_states)

    def init_ec_sdk(self) -> None:
        if self.use_fake:
            self.elite_robot = EliteArmFakeEc(
                ip_address=self.ip_address, auto_connect=self.auto_connect)
        else:
            self.elite_robot = EC(
                ip=self.ip_address, auto_connect=self.auto_connect)
            if self.elite_robot.state == EC.RobotState.PLAY:
                self.elite_robot.stop()
            self.elite_robot.robot_servo_on()
        self.elite_robot.monitor_thread_run()
        while self.elite_robot.monitor_info.machinePos[0] == None:
            print("Monitor is not start,wait...")
            time.sleep(1)

        print("Robot startup success...")

    def publish_states(self):
        EliteStatePublisher.update_states(self)

def main(args=None):
    rclpy.init(args=args)

    elite_driver = EliteDriver()

    # Initialize elite_drivers
    elite_driver.init_ec_sdk()

    rclpy.spin(elite_driver)

if __name__ == "__main__":
    main()
