#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from elite_msgs.srv import StopMove
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from .interpolate_five import get_five_fun, get_path_fun
import math
import time

class EliteArmTrajectoryAction():
    def __init__(self, ):
        self.get_logger().info("starting elite arm trajectory action")
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback)
        self.interpolate_sample_time = 0.008
        self.stop_mov_cli = self.create_client(StopMove, 'stop_move_server')
        self.goal_handle_ = None

    def goal_callback(self, goal_):
        self.get_logger().info('Goal request recieved')
        self.goal = goal_
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info("Stopping initiated")
        self._stop_move()
        self.goal_handle = None
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        # Takes care of multiple goal requests
        if self.goal_handle_ != None:
            if self.goal_handle_.is_active:
                self.get_logger().info("Stopping the executing goal")
                self._stop_move()
                self.goal_handle_.canceled()

        self.goal_handle_ = goal_handle
        self.get_logger().info("Executing the new goal")
        self.goal_handle_.execute()

    def execute_callback(self, goal_handle):
        points = self.goal.trajectory.points
        joints = []
        joint = []
        time_stamp = []
        for joint_index in range(6):
            sum_time, f, s, a = get_path_fun(points, joint_index)
            jointi = []
            length = int(sum_time/self.interpolate_sample_time)
            for point_index in range(length):
                jointi.append(math.degrees(
                    f(self.interpolate_sample_time*point_index)))
            joints.append(jointi)

        length = len(joints[0])

        for point_index in range(length):
            time_stamp.append(point_index*self.interpolate_sample_time)
            for i in range(6):
                joint.append(joints[i][point_index])
            joint.append(0.0)
            joint.append(0.0)

        self.elite_robot.TT_init(t=time_stamp[1]*1000)
        last_joint = []
        for i in range(len(time_stamp)):
            # check if cancellation is requested
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Returning stop from the execution")
                result = FollowJointTrajectory.Result()
                result.error_code = result.SUCCESSFUL
                result.error_string = "Cancelled"
                
                self.goal_handle_ = None
                self.elite_robot.TT_clear_buff()
                return result

            temp_joint = joint[i*8:i*8+6]
            self.elite_robot.TT_add_joint(temp_joint)
            last_joint = temp_joint

        while 1:
            time.sleep(0.1)
            self.elite_robot:EC
            current_joint = [round(i,1) for i in self.elite_robot.current_joint]
            goal_joint = [round(i,1) for i in last_joint]
            if (current_joint == goal_joint):
                self.elite_robot.TT_clear_buff()
                break

        goal_handle.succeed()

        # reset the global goal_handle
        self.goal_handle_ = None

        # ToDo: Handle errors and failures
        result = FollowJointTrajectory.Result()
        result.error_code = result.SUCCESSFUL
        result.error_string = "Completed"

        self.get_logger().info("FollowJointTrajectory Completed the Goal")

        return result

    def _stop_move(self) -> bool:
        request = StopMove.Request()
        self.future = self.stop_mov_cli.call_async(request)
        time.sleep(1)
        return True
