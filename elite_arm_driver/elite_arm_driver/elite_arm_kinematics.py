#!/usr/bin/env python3
import rclpy
from elite_msgs.srv import ForwardKinematic, InverseKinematic
import transform3d as tfs
import math

class EliteArmKinematics():
    def __init__(self):
        self.get_logger("Listening for kinematics request")
        self.fk_srv = self.create_service(ForwardKinematic, 'forward_kinematics', self.forward_kinematics_cb)
        self.ik_srv = self.create_service(InverseKinematic, 'inverse_kinematics', self.inverse_kinematics_cb)

    def forward_kinematics_cb(self, request, response):
        result = self.elite_robot.get_forward_kinematic(request.joint)
        response.pose.position.x = result[0]
        response.pose.position.y = result[1]
        response.pose.position.z = result[2]
        quat = tfs.euler.euler2quat(result[3], result[4], result[5])
        response.pose.orientation.w = quat[0]
        response.pose.orientation.x = quat[1]
        response.pose.orientation.y = quat[2]
        response.pose.orientation.z = quat[3]
        return response

    def inverse_kinematics_cb(self, request, response):
        target_point = []
        target_point.append(request.pose.position.x)
        target_point.append(request.pose.position.y)
        target_point.append(request.pose.position.z)

        euler_angle = tfs.euler.quat2euler(
            [request.pose.orientation.w, request.pose.orientation.x,
             request.pose.orientation.y, request.pose.orientation.z])
        for angle in euler_angle:
          target_point.append(math.degrees(angle))

        result = self.elite_robot.get_inverse_kinematic(
            target_point, request.ref_joint, unit_type=0)
        if type(result) == tuple:
            response.result = False
        else:
            response.joint = result
            response.result = True

        return response  