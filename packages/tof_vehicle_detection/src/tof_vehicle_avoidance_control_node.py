#!/usr/bin/env python
from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped, BoolStamped, ToFStamped
import time

import rospy


class ToFVehicleAvoidanceControlNode(DTROS):

	def __init__(self, node_name="tof_vehicle_avoidance_control_node"):
		super(ToFVehicleAvoidanceControlNode, self).__init__(node_name=node_name)
		print("########### TOF VEHICLE AVOIDANCE CONTROL ##########")

		self.parameters['~desired_distance'] = None
		self.parameters['~detection_min_distance'] = None
		self.parameters['~detection_max_distance'] = None
		self.parameters['~distance_avg'] = None
		self.parameters['~Kp'] = None
		self.parameters['~Ki'] = None
		self.parameters['~Kd'] = None
		self.parameters['~Kp_delta_v'] = None
		self.updateParameters()

		self.distance_measurements = [0] * self.parameters['~distance_avg']
		self.prev_avg_distance = 0
		self.curr_avg_distance = 0
		self.prev_time = time.time()
		self.d_time = 0
		self.detection = None

		self.error = 0
		self.derivative = 0
		self.integral = 0
		
		self.car_cmd_pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
		self.vehicle_detected_pub = rospy.Publisher("~vehicle_detected", BoolStamped, queue_size=1)

		self.sub_tof = rospy.Subscriber("tof_node/tof_5", ToFStamped, self.tof_callback, queue_size=1)
		self.sub_car_cmd = rospy.Subscriber("~car_cmd_in", Twist2DStamped, self.car_cmd_callback, queue_size=1)

	def tof_callback(self, msg):
		self.distance_measurements = self.distance_measurements[1:] + [msg.distance / 1000.0]
		self.prev_avg_distance = self.curr_avg_distance
		self.curr_avg_distance = sum(self.distance_measurements) / len(self.distance_measurements)
		self.error = self.curr_avg_distance - self.parameters['~desired_distance']

		now = time.time()
		self.d_time = now - self.prev_time
		self.prev_time = now
		self.derivative = (self.curr_avg_distance - self.prev_avg_distance) / self.d_time
		self.integral += self.error * self.d_time

		if self.curr_avg_distance < self.parameters['~detection_min_distance']:
			self.detection = True
		elif self.curr_avg_distance > self.parameters['~detection_max_distance']:
			self.detection = False
			self.integral = 0

		self.log("Distance: {}, detect: {}, Err: {}, I: {}, D: {}".format(self.curr_avg_distance, self.detection, self.error, self.integral, self.derivative))

	def car_cmd_callback(self, car_cmd):
		new_v = \
			self.parameters['~Kp'] * self.error + \
			self.parameters['~Ki'] * self.integral + \
			self.parameters['~Kd'] * self.derivative
		new_v = max(0, new_v)

		if self.detection:
			car_cmd.v = min(max(new_v, 0), car_cmd.v)

		self.car_cmd_pub.publish(car_cmd)

		detected = BoolStamped()
		detected.header.stamp = rospy.Time.now()
		detected.data = self.detection
		self.vehicle_detected_pub.publish(detected)


if __name__ == "__main__":
	node = ToFVehicleAvoidanceControlNode()
	rospy.spin()
