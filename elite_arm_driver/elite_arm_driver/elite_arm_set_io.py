#!/usr/bin/env python3
import rclpy
from elite_msgs.srv import SetAnalogIO, SetIO

class EliteArmSetIO():
    def __init__(self):
        print("starting elite arm set IO")
        self.digital_io_srv = self.create_service(SetIO, 'set_digital_io', self.digital_io_cb)
        self.analog_io_srv = self.create_service(SetAnalogIO, 'set_analog_io', self.analog_io_cb)

    def digital_io_cb(self, request, response):
        response = self.elite_robot.set_analog_output(  # pylint: disable=E1101
            request.address, request.value)
        response.result = result
        print(f"result:{result}")
        return response

    def analog_io_cb(self, request, response):
        result = self.elite_robot.set_analog_output(  # pylint: disable=E1101
            request.address, request.value)
        response.result = result
        print(f"result:{result}")
        return response
