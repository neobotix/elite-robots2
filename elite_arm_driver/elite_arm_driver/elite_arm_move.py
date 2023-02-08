#!/usr/bin/env python3
import rclpy
from elite_msgs.srv import CartMove, JointMove, StopMove

class EliteArmMove:
    def __init__(self, ):
        self.get_logger().info("ready to move arm")
        self.cart_mov_srv = self.create_service(CartMove, 'cart_move_server', self.cart_move_cb)
        self.joint_mov_srv = self.create_service(JointMove, 'joint_move_server', self.joint_move_cb)
        self.stop_mov_srv = self.create_service(StopMove, 'stop_move_server', self.stop_move_cb)

    def cart_move_cb(self, request, response):
        target_joint =  request.target_joint
        speed = request.speed
        speed_type = request.speed_type
        acc = request.acc
        dec = request.dec        
        is_block = request.is_blocking
        result_ = self.elite_robot.move_line(  # pylint: disable=E1101
            target_joint, speed, speed_type, acc, dec)
        if is_block:
            self.elite_robot.wait_stop()  # pylint: disable=E1101
        response.result = result_
        return response

    def joint_move_cb(self, request, response):
        joint_point_ = request.target_joint
        speed_ = request.speed
        acc_ = request.acc
        dec_ = request.dec
        is_block_ = request.is_blocking
        result_ = self.elite_robot.move_joint(  # pylint: disable=E1101
            joint_point_, speed_, acc_, dec_)
        if is_block_:
            self.elite_robot.wait_stop()  # pylint: disable=E1101
        if type(result_) == bool:
            response.result = result_
        else:
            response.result = result_[0]
        return response

    def stop_move_cb(self, request, response):
        self.get_logger().info("stop_move_server recieved command to stop the robot")
        result_ = self.elite_robot.stop()  # pylint: disable=E1101
        response.result = result_
        # ToDo: Log the response
        return response 
