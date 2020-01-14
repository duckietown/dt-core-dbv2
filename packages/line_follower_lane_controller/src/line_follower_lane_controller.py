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

    def line_sensor_callback(self, msg):
        current_time = time.time()
        duration = current_time - self.last_iteration_time
        detections = self.threshold_measurement(msg)
        if not detections.inner_right and not detections.outer_right:
            # Too far left
            current_state = -1
        elif not detections.inner_right and detections.outer_right:
            # Perfect
            current_state = 0
        elif detections.inner_right and detections.outer_right:
            # Too far right
            current_state = 1
        elif detections.inner_right and not detections.outer_right:
            # WAY too far right
            current_state = 2

        self.integral = self.parameters['~I_decay'] * (self.integral + current_state * duration)
        self.integral = min(self.parameters['~max_I'], max(-self.parameters['~max_I'], self.integral))

        self.log("State: {}, Integral: {}, Duration: {}".format(current_state, self.integral, duration))

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
