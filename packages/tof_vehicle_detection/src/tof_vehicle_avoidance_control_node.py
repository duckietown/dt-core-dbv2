#!/usr/bin/env python
import time
from enum import Enum
from collections import namedtuple
import rospy
from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped, BoolStamped, ToFStamped


class Sensor(Enum):
    LEFT = 1
    MIDDLE = 2
    RIGHT = 3


class ToFVehicleAvoidanceControlNode(DTROS):

    def __init__(self, node_name="tof_vehicle_avoidance_control_node"):
        super(ToFVehicleAvoidanceControlNode, self).__init__(node_name=node_name)
        print("########### TOF VEHICLE AVOIDANCE CONTROL ##########")

        self.parameters['~desired_distance'] = None
        self.parameters['~detection_min_distance'] = None
        self.parameters['~detection_max_distance'] = None
        self.parameters['~distance_avg'] = None
        self.parameters['~omega_min'] = None
        self.parameters['~omega_max'] = None
        self.parameters['~Kp'] = None
        self.parameters['~Ki'] = None
        self.parameters['~Kd'] = None
        self.parameters['~Kp_delta_v'] = None
        self.parameters['~max_speed'] = None
        self.updateParameters()

        self.distance_measurements = [0] * self.parameters['~distance_avg']
        self.prev_avg_distance = 0
        self.prev_time = time.time()
        self.detection = False

        self.left_sensor_measurements = []
        self.middle_sensor_measurements = []
        self.right_sensor_measurements = []

        self.error = 0
        self.derivative = 0
        self.integral = 0

        self.current_car_cmd = Twist2DStamped()

        self.car_cmd_pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.vehicle_detected_pub = rospy.Publisher("~vehicle_detected", BoolStamped, queue_size=1)
        self.sub_car_cmd = rospy.Subscriber("~car_cmd_in", Twist2DStamped, self.car_cmd_callback, queue_size=1)

        self.sub_tof_middle = rospy.Subscriber(
            "tof_node/tof_5",
            ToFStamped,
            queue_size=1,
            callback=self.tof_callback,
            callback_args=self.middle_sensor_measurements
        )

        self.sub_tof_left = rospy.Subscriber(
            "tof_node/tof_6",
            ToFStamped,
            queue_size=1,
            callback=self.tof_callback,
            callback_args=self.left_sensor_measurements
        )

        self.sub_tof_right = rospy.Subscriber(
            "tof_node/tof_1",
            ToFStamped,
            queue_size=1,
            callback=self.tof_callback,
            callback_args=self.right_sensor_measurements
        )

    def tof_callback(self, msg, measurements):
        if msg.error == 0:
            measurements.append(msg.distance / 1000.0)
        if len(measurements) > self.parameters['~distance_avg']:
            del measurements[0]
        if measurements is self.middle_sensor_measurements \
                and len(self.left_sensor_measurements) > 0 \
                and len(self.right_sensor_measurements) > 0:
            self.tof_measurement_update()

    def tof_measurement_update(self):

        front_distance = sum(self.middle_sensor_measurements) / len(self.middle_sensor_measurements)
        if self.current_car_cmd.omega < 0:
            inner_side_measurements = self.right_sensor_measurements
            outer_side_measurements = self.left_sensor_measurements
        else:
            inner_side_measurements = self.left_sensor_measurements
            outer_side_measurements = self.right_sensor_measurements
        inner_side_distance = sum(inner_side_measurements) / len(inner_side_measurements)
        outer_side_distance = sum(outer_side_measurements) / len(outer_side_measurements)

        omega_min = self.parameters['~omega_min']
        omega_max = self.parameters['~omega_max']
        # omega <= omega_min: mix = 0
        # omega >= omega_max: mix = 1
        # omega_min < omega < omega_max: min is mapped linearly between 0 and 1
        mix = min(max((self.current_car_cmd.omega - omega_min) / (omega_max - omega_min), 0), 1)

        distance = (mix * inner_side_distance) + ((1.0 - mix) * front_distance)

        self.error = distance - self.parameters['~desired_distance']

        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now
        self.derivative = (distance - self.prev_avg_distance) / dt
        self.integral += self.error * dt
        self.prev_avg_distance = distance

        if distance < self.parameters['~detection_min_distance']:
            self.detection = True
        elif distance > self.parameters['~detection_max_distance']:
            self.detection = False
            self.integral = 0

        # self.log("Distance: {}, detect: {}, Err: {}, I: {}, D: {}".format(distance, self.detection,
        #          self.error, self.integral, self.derivative))

    def car_cmd_callback(self, car_cmd):

        new_v = \
            self.parameters['~Kp'] * self.error + \
            self.parameters['~Ki'] * self.integral + \
            self.parameters['~Kd'] * self.derivative
        new_v = max(0, new_v)

        if self.detection:
            car_cmd.v = min(max(new_v, 0), self.parameters['~max_speed'])

        self.current_car_cmd = car_cmd
        self.car_cmd_pub.publish(car_cmd)

        detected = BoolStamped()
        detected.header.stamp = rospy.Time.now()
        detected.data = self.detection
        self.vehicle_detected_pub.publish(detected)


if __name__ == "__main__":
    node = ToFVehicleAvoidanceControlNode()
    rospy.spin()
