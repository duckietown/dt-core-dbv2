#!/usr/bin/env python

from __future__ import print_function

import math
import time
import numpy as np
import rospy
from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped, LineFollowerStamped
import time
import math
from collections import namedtuple


LineDetected = namedtuple("LineDetected", ("outer_right", "inner_right", "inner_left", "outer_left"))


class LineFollowerLaneController(DTROS):

    def __init__(self, node_name="line_follower_lane_controller"):
        super(LineFollowerLaneController, self).__init__(node_name=node_name)

        self.parameters['~steering_gain'] = None
        self.parameters['~k_I'] = None
        self.parameters['~max_I'] = None
        self.parameters['~I_decay'] = None
        self.parameters['~update_hz'] = None
        self.parameters['~drive_speed'] = None
        self.parameters['~outer_right_threshold'] = None
        self.parameters['~inner_right_threshold'] = None
        self.parameters['~inner_left_threshold'] = None
        self.parameters['~outer_left_threshold'] = None
        self.updateParameters()

        self.last_iteration_time = time.time()

        # The current state is:
        #  -1: Too far left (Both sensors detect black)
        #   0: Good (Inner sensor detects black, outer sensor detects white)
        #  +1: Too far right (Both sensors detect white)
        #  +2: Way too far right, outside of white line (Inner sensor detects white, outer sensor detects black)
        self.prevState = 0

        self.integral = 0

        self.sub = self.subscriber(
            "line_following_node/line_follower", LineFollowerStamped, self.line_sensor_callback, queue_size=1
        )

        self.car_cmd_pub = self.publisher(
            "~car_cmd", Twist2DStamped, queue_size=1
        )

    def line_sensor_callback(self, msg):
        current_time = time.time()
        duration = current_time - self.last_iteration_time
        detections = self.threshold_measurement(msg)

        # This dict maps line detections to a number representing the current state
        states = {
            # (Black, Black): Too far left
            (False, False): -1,
            # (Black, White): Perfectly on the line
            (False, True): 0,
            # (White, White): Too far right
            (True, True): 1,
            # (White, Black): Way too far right (past the line)
            (True, False): 2
        }

        current_state = states[(detections.inner_right, detections.outer_right)]

        self.integral = self.parameters['~I_decay'] * (self.integral + current_state * duration)
        # Constrain the integral to within +/- max_I
        self.integral = min(self.parameters['~max_I'], max(-self.parameters['~max_I'], self.integral))

        omega = self.parameters['~steering_gain'] * current_state + self.parameters['~k_I'] * self.integral

        self.log("State: {}, Integral: {}, Omega: {}, Raw measurements: ({}, {})".format(
            current_state, self.integral, omega, msg.inner_right, msg.outer_right))

        car_cmd = Twist2DStamped()
        car_cmd.header.stamp = rospy.get_rostime()
        car_cmd.v = self.parameters['~drive_speed']
        car_cmd.omega = omega
        self.car_cmd_pub.publish(car_cmd)

        self.prevState = current_state
        self.last_iteration_time = current_time

    def threshold_measurement(self, msg):
        return LineDetected(
            outer_right=msg.outer_right < self.parameters['~outer_right_threshold'],
            inner_right=msg.inner_right < self.parameters['~inner_right_threshold'],
            outer_left=msg.outer_left < self.parameters['~outer_left_threshold'],
            inner_left=msg.inner_left < self.parameters['~inner_left_threshold']
        )


if __name__ == "__main__":
    lane_control_node = LineFollowerLaneController()
    rospy.spin()
