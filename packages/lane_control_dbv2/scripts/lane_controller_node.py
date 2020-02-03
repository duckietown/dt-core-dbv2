#!/usr/bin/env python

from __future__ import print_function

import math
import time
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdDBV2Stamped, BoolStamped, FSMState, StopLineReading, ParamTuner
import time
import control

class lane_controller(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = None
        self.last_ms = None
        self.pub_counter = 0
        self.outputs = [0] * 3
        self.velocity_to_m_per_s = 0.67
        self.omega_to_rad_per_s = 0.45 * 2 * math.pi

        # Setup parameters
        self.velocity_to_m_per_s = 1.53
        self.omega_to_rad_per_s = 4.75
        self.setGains()

        # Publication
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_actuator_limits_received = rospy.Publisher("~actuator_limits_received", BoolStamped, queue_size=1)
        self.pub_radius_limit = rospy.Publisher("~radius_limit", BoolStamped, queue_size=1)
        self.pub_param_tuner = rospy.Publisher("~param_tuner", ParamTuner, queue_size=1)



        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.PoseHandling, "lane_filter", queue_size=1)

        self.sub_obstacle_avoidance_pose = rospy.Subscriber("~obstacle_avoidance_pose", LanePose, self.PoseHandling, "obstacle_avoidance",queue_size=1)
        self.sub_obstacle_detected = rospy.Subscriber("~obstacle_detected", BoolStamped, self.setFlag, "obstacle_detected", queue_size=1)

        self.sub_intersection_navigation_pose = rospy.Subscriber("~intersection_navigation_pose", LanePose, self.PoseHandling, "intersection_navigation",queue_size=1)   # TODO: remap topic in file catkin_ws/src/70-convenience-packages/duckietown_demos/launch/master.launch !



        # self.sub_parking_pose = rospy.Subscriber("~parking_pose", LanePose, self.PoseHandling, "parking",queue_size=1)   # TODO: remap topic in file catkin_ws/src/70-convenience-packages/duckietown_demos/launch/master.launch !

        # TO DO find node/topic in their branch
        # self.sub_fleet_planning_pose = rospy.Subscriber("~fleet_planning", LanePose, self.PoseHandling, "fleet_planning",queue_size=1)   # TODO: remap topic in file catkin_ws/src/70-convenience-packages/duckietown_demos/launch/master.launch !
        # self.sub_fleet_planning_lane_following_override_active = rospy.Subscriber("~fleet_planning_lane_following_override_active", BoolStamped, self.setFlag, "fleet_planning_lane_following_override_active", queue_size=1)   # TODO: remap topic in file catkin_ws/src/70-convenience-packages/duckietown_demos/launch/master.launch !

        # TO DO find node/topic in their branch
        # self.sub_implicit_coord_pose = rospy.Subscriber("~implicit_coord", LanePose, self.PoseHandling, "implicit_coord",queue_size=1)   # TODO: remap topic in file catkin_ws/src/70-convenience-packages/duckietown_demos/launch/master.launch !
        # self.sub_implicit_coord_velocity_limit_active = rospy.Subscriber("~implicit_coord_velocity_limit_active", BoolStamped, self.setFlag, "implicit_coord_velocity_limit_active", queue_size=1)   # TODO: remap topic in file catkin_ws/src/70-convenience-packages/duckietown_demos/launch/master.launch !

        self.sub_wheels_cmd_executed = rospy.Subscriber("~wheels_cmd_executed", WheelsCmdDBV2Stamped, self.updateWheelsCmdDBV2Executed, queue_size=1)
        self.sub_actuator_limits = rospy.Subscriber("~actuator_limits", Twist2DStamped, self.updateActuatorLimits, queue_size=1)

        # FSM
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch,  queue_size=1)     # for this topic, no remapping is required, since it is directly defined in the namespace lane_controller_node by the fsm_node (via it's default.yaml file)


        self.sub_stop_line = rospy.Subscriber("~stop_line_reading",StopLineReading, self.cbStopLineReading,  queue_size=1)     # for this topic, no remapping is required, since it is directly defined in the namespace lane_controller_node by the fsm_node (via it's default.yaml file)


        self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)

        self.msg_radius_limit = BoolStamped()
        self.msg_radius_limit.data = self.use_radius_limit
        self.pub_radius_limit.publish(self.msg_radius_limit)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(0.1), self.getGains_event)
        rospy.loginfo("[%s] Initialized (LQRI version)" % (rospy.get_name()))

        self.stop_line_distance = 999
        self.stop_line_detected = False

        #LQR control
        self.d_int = 0.
        #in param file: self.q_u = np.array([[100., 0, 0], [0, 2., 0], [0, 0, 120.]])
        #in param file: self.Norm = np.array([[0.0083,0,0],[0,0.0083,0],[0,0,0.0083]])
        #print((self.q_u,self.Norm))
        self.q = self.q_u*self.Norm
        #in param file: self.r = np.array([10])
        #in param file: self.lqr_gain = 0.8
        #smoothing of control input for phi
        self.prev_heading_err = 0
        self.prev_prev_heading_err = 0
        self.ppp_heading_err = 0
        self.pppp_heading_err = 0

    def cbStopLineReading(self, msg):
        self.stop_line_distance = np.sqrt(msg.stop_line_point.x**2 + msg.stop_line_point.y**2 + msg.stop_line_point.z**2)
        self.stop_line_detected = msg.stop_line_detected

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value)   # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def setGains(self):
        self.v_bar_gain_ref = 0.5
        v_bar_fallback = 0.25  # nominal speed, 0.25m/s
        self.v_max = 1
        k_theta_fallback = -2.0
        k_d_fallback = - (k_theta_fallback ** 2) / (4.0 * self.v_bar_gain_ref)
        theta_thres_fallback = math.pi / 6.0
        d_thres_fallback = math.fabs(k_theta_fallback / k_d_fallback) * theta_thres_fallback
        d_offset_fallback = 0.0

        k_theta_fallback = k_theta_fallback
        k_d_fallback = k_d_fallback

        k_Id_fallback = 2.5
        k_Iphi_fallback = 1.25

        k_Dd_fallback = 0.0
        k_Dphi_fallback = 0.0

        #LQRI
        q_u_fallback = np.array([[10000., 0, 0], [0, 2., 0], [0, 0, 20000.]])
        q_u_fallback_list = np.reshape(q_u_fallback, [-1]).tolist()
        Norm_fallback = np.array([[0.00005,0,0],[0,0.00005,0],[0,0,0.00005]])
        Norm_fallback_list = np.reshape(Norm_fallback, [-1]).tolist()
        r_fallback = np.array([0.078])
        r_fallback_list = np.reshape(r_fallback, [-1]).tolist()
        lqr_gain_fallback = 1.0

        self.fsm_state = None
        self.cross_track_err = 0
        self.heading_err = 0
        self.cross_track_integral = 0
        self.heading_integral = 0
        self.cross_track_integral_top_cutoff = 0.3
        self.cross_track_integral_bottom_cutoff = -0.3
        self.heading_integral_top_cutoff = 1.2
        self.heading_integral_bottom_cutoff = -1.2
        #-1.2
        self.time_start_curve = 0

        use_feedforward_part_fallback = False
        self.wheels_cmd_executed = WheelsCmdDBV2Stamped()

        self.actuator_limits = Twist2DStamped()
        self.actuator_limits.v = 999.0  # to make sure the limit is not hit before the message is received
        self.actuator_limits.omega = 999.0  # to make sure the limit is not hit before the message is received
        self.omega_max = 999.0  # considering radius limitation and actuator limits   # to make sure the limit is not hit before the message is received

        self.use_radius_limit_fallback = True

        self.flag_dict = {"obstacle_detected": False,
                          "parking_stop": False,
                          "fleet_planning_lane_following_override_active": False,
                          "implicit_coord_velocity_limit_active": False}

        self.pose_msg = LanePose()
        self.pose_initialized = False
        self.pose_msg_dict = dict()
        self.v_ref_possible = dict()
        self.main_pose_source = None

        self.active = True

        self.sleepMaintenance = False

        # overwrites some of the above set default values (the ones that are already defined in the corresponding yaml-file (see launch-file of this node))

        self.v_bar = self.setupParameter("~v_bar",v_bar_fallback)   # Linear velocity
        self.k_d = self.setupParameter("~k_d",k_d_fallback)     # P gain for d
        self.k_theta = self.setupParameter("~k_theta",k_theta_fallback)     # P gain for theta
        self.d_thres = self.setupParameter("~d_thres",d_thres_fallback)     # Cap for error in d
        self.theta_thres = self.setupParameter("~theta_thres",theta_thres_fallback)     # Maximum desire theta
        self.d_offset = self.setupParameter("~d_offset",d_offset_fallback)  # a configurable offset from the lane position

        self.k_Id = self.setupParameter("~k_Id", k_Id_fallback)     # gain for integrator of d
        self.k_Iphi = self.setupParameter("~k_Iphi",k_Iphi_fallback)    # gain for integrator of phi (phi = theta)

        self.k_Dd = self.setupParameter("~k_Dd", k_Dd_fallback)
        self.k_Dphi = self.setupParameter("~k_Dphi", k_Dphi_fallback)

        #print(q_u_fallback)
        #Parameters for LQRI control
        self.q_u = np.reshape(np.array(self.setupParameter("~q_u", q_u_fallback_list)), q_u_fallback.shape)
        self.Norm = np.reshape(np.array(self.setupParameter("~Norm", Norm_fallback_list)), Norm_fallback.shape)
        self.r = np.reshape(np.array(self.setupParameter("~r", r_fallback_list)), r_fallback.shape)
        self.lqr_gain = self.setupParameter("~lqr_gain", lqr_gain_fallback)

        #TODO: Feedforward was not working, go away with this error source! (Julien)
        self.use_feedforward_part = self.setupParameter("~use_feedforward_part",use_feedforward_part_fallback)
        self.omega_ff = self.setupParameter("~omega_ff",0)
        self.omega_max = self.setupParameter("~omega_max", 999)
        self.omega_min = self.setupParameter("~omega_min", -999)
        self.use_radius_limit = self.setupParameter("~use_radius_limit", self.use_radius_limit_fallback)
        self.min_radius = self.setupParameter("~min_rad", 0.0)

        self.d_ref = self.setupParameter("~d_ref", 0)
        self.phi_ref = self.setupParameter("~phi_ref",0)
        self.object_detected = self.setupParameter("~object_detected", 0)
        self.v_ref_possible["default"] = self.v_max


    def getGains_event(self, event):
        v_bar = rospy.get_param("~v_bar")
        k_d = rospy.get_param("~k_d")
        k_theta = rospy.get_param("~k_theta")
        k_Dd = rospy.get_param("~k_Dd")
        k_Dphi = rospy.get_param("~k_Dphi")
        d_thres = rospy.get_param("~d_thres")
        theta_thres = rospy.get_param("~theta_thres")
        d_offset = rospy.get_param("~d_offset")
        d_ref = rospy.get_param("~d_ref")
        phi_ref = rospy.get_param("~phi_ref")
        use_radius_limit = rospy.get_param("~use_radius_limit")
        object_detected = rospy.get_param("~object_detected")
        self.omega_ff = rospy.get_param("~omega_ff")
        self.omega_max = rospy.get_param("~omega_max")
        self.omega_min = rospy.get_param("~omega_min")
        #FeedForward
        #TODO: Feedforward was not working, go away with this error source! (Julien)

        self.velocity_to_m_per_s = 1#0.67     # TODO: change after new kinematic calibration! (should be obsolete, if new kinematic calibration works properly)
        #self.omega_to_rad_per_s = 0.45 * 2 * math.pi    # TODO: change after new kinematic calibration! (should be obsolete, if new kinematic calibration works properly)

        # FeedForward
        self.curvature_outer = 1 / (0.39)
        self.curvature_inner = 1 / 0.175

        use_feedforward_part = rospy.get_param("~use_feedforward_part")


        k_Id = rospy.get_param("~k_Id")
        k_Iphi = rospy.get_param("~k_Iphi")
        if self.k_Id != k_Id:
            rospy.loginfo("ADJUSTED I GAIN")
            self.cross_track_integral = 0
            self.k_Id = k_Id


        #LQRI part:
        q_u = np.array(rospy.get_param("~q_u"))
        Norm = np.array(rospy.get_param("~Norm"))
        r = rospy.get_param("~r")
        lqr_gain = rospy.get_param("~lqr_gain")

        params_old = (self.v_bar,self.k_d,self.k_theta,self.d_thres,self.theta_thres, self.d_offset, self.k_Id, self.k_Iphi, self.k_Dd, self.k_Dphi, self.use_feedforward_part, self.use_radius_limit)
        params_new = (v_bar,k_d,k_theta,d_thres,theta_thres, d_offset, k_Id, k_Iphi, k_Dd, k_Dphi, use_feedforward_part, use_radius_limit)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." %(self.node_name))

            self.v_bar = v_bar
            self.k_d = k_d
            self.k_theta = k_theta
            self.d_thres = d_thres
            self.d_ref = d_ref
            self.phi_ref = phi_ref
            self.theta_thres = theta_thres
            self.d_offset = d_offset
            self.k_Id = k_Id
            self.k_Iphi = k_Iphi
            self.k_Dd = k_Dd
            self.k_Dphi = k_Dphi

            self.use_feedforward_part = use_feedforward_part


            if use_radius_limit != self.use_radius_limit:
                self.use_radius_limit = use_radius_limit
                self.msg_radius_limit.data = self.use_radius_limit
                self.pub_radius_limit.publish(self.msg_radius_limit)

    # FSM

    def cbSwitch(self,fsm_switch_msg):
        self.active = fsm_switch_msg.data   # True or False

        rospy.loginfo("active: " + str(self.active))
    # FSM

    def unsleepMaintenance(self, event):
        self.sleepMaintenance = False

    def cbMode(self,fsm_state_msg):

        # if self.fsm_state != fsm_state_msg.state and fsm_state_msg.state == "IN_CHARGING_AREA":
        #     self.sleepMaintenance = True
        #     self.sendStop()
        #     rospy.Timer(rospy.Duration.from_sec(2.0), self.unsleepMaintenance)

        self.fsm_state = fsm_state_msg.state    # String of current FSM state
        print("fsm_state changed in lane_controller_node to: " , self.fsm_state)

    def setFlag(self, msg_flag, flag_name):
        self.flag_dict[flag_name] = msg_flag.data
        if flag_name == "obstacle_detected":
             print("flag obstacle_detected changed")
             print("flag_dict[\"obstacle_detected\"]: ", self.flag_dict["obstacle_detected"])


    def PoseHandling(self, input_pose_msg, pose_source):
        if not self.active:
            return

        if self.sleepMaintenance:
            return

        #if pose_source == "obstacle_avoidance":
            # print "obstacle_avoidance pose_msg d_ref: ", input_pose_msg.d_ref
            # print "obstacle_avoidance pose_msg v_ref: ", input_pose_msg.v_ref
            # print "flag_dict[\"obstacle_detected\"]: ", self.flag_dict["obstacle_detected"]

        self.prev_pose_msg = self.pose_msg
        self.pose_msg_dict[pose_source] = input_pose_msg
        # self.fsm_state = "INTERSECTION_CONTROL" #TODO pass this message automatically
        if self.pose_initialized:
            v_ref_possible_default = self.v_ref_possible["default"]
            v_ref_possible_main_pose = self.v_ref_possible["main_pose"]
            self.v_ref_possible.clear()
            self.v_ref_possible["default"] = v_ref_possible_default
            self.v_ref_possible["main_pose"] = v_ref_possible_main_pose

        if self.fsm_state == "INTERSECTION_CONTROL":
            if pose_source == "intersection_navigation": # for CL intersection from AMOD use 'intersection_navigation'
                self.pose_msg = input_pose_msg
                self.pose_msg.curvature_ref = input_pose_msg.curvature
                self.v_ref_possible["main_pose"] = self.v_bar
                self.main_pose_source = pose_source
                self.pose_initialized = True
        elif self.fsm_state == "PARKING":
            if pose_source == "parking":
                #rospy.loginfo("pose source: parking!?")
                self.pose_msg = input_pose_msg
                self.v_ref_possible["main_pose"] = input_pose_msg.v_ref
                self.main_pose_source = pose_source
                self.pose_initialized = True
        else:
            if pose_source == "lane_filter":
                #rospy.loginfo("pose source: lane_filter")
                self.pose_msg = input_pose_msg
                self.pose_msg.curvature_ref = input_pose_msg.curvature


                self.v_ref_possible["main_pose"] = self.v_bar

                # Adapt speed to stop line!
                if self.stop_line_detected:
                    # 60cm -> v_bar, 15cm -> v_bar/2
                    d1, d2 = 0.8, 0.25
                    a = self.v_bar/(2*(d1-d2))
                    b = self.v_bar - a*d1
                    v_new = a*self.stop_line_distance + b
                    v_new = np.max([self.v_bar/2.0, np.min([self.v_bar, v_new])])
                    self.v_ref_possible["main_pose"] = v_new
                self.main_pose_source = pose_source
                self.pose_initialized = True

        if self.flag_dict["fleet_planning_lane_following_override_active"] == True:
            if "fleet_planning" in self.pose_msg_dict:
                self.pose_msg.d_ref = self.pose_msg_dict["fleet_planning"].d_ref
                self.v_ref_possible["fleet_planning"] = self.pose_msg_dict["fleet_planning"].v_ref
        if self.flag_dict["obstacle_detected"] == True:
            if "obstacle_avoidance" in self.pose_msg_dict:
                self.pose_msg.d_ref = self.pose_msg_dict["obstacle_avoidance"].d_ref
                self.v_ref_possible["obstacle_avoidance"] = self.pose_msg_dict["obstacle_avoidance"].v_ref
                #print 'v_ref obst_avoid=' , self.v_ref_possible["obstacle_avoidance"] #For debugging
        if self.flag_dict["implicit_coord_velocity_limit_active"] == True:
            if "implicit_coord" in self.pose_msg_dict:
                self.v_ref_possible["implicit_coord"] = self.pose_msg_dict["implicit_coord"].v_ref

        self.pose_msg.v_ref = min(self.v_ref_possible.itervalues())
        #print 'v_ref global=', self.pose_msg.v_ref #For debugging

        if self.pose_msg != self.prev_pose_msg and self.pose_initialized:
            self.cbPose(self.pose_msg)

    def updateWheelsCmdDBV2Executed(self, msg_wheels_cmd):
        self.wheels_cmd_executed = msg_wheels_cmd

    def updateActuatorLimits(self, msg_actuator_limits):
        self.actuator_limits = msg_actuator_limits
        rospy.loginfo("actuator limits updated to: ")
        rospy.loginfo("actuator_limits.v: " + str(self.actuator_limits.v))
        rospy.loginfo("actuator_limits.omega: " + str(self.actuator_limits.omega))
        msg_actuator_limits_received = BoolStamped()
        msg_actuator_limits_received.data = True
        self.pub_actuator_limits_received.publish(msg_actuator_limits_received)

    def sendStop(self):
        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)

        # Stop listening
        self.sub_lane_reading.unregister()
        self.sub_obstacle_avoidance_pose.unregister()
        self.sub_obstacle_detected.unregister()
        self.sub_intersection_navigation_pose.unregister()
        # self.sub_parking_pose.unregister()
        # self.sub_fleet_planning_pose.unregister()
        # self.sub_fleet_planning_lane_following_override_active.unregister()
        # self.sub_implicit_coord_pose.unregister()
        # self.sub_implicit_coord_velocity_limit_active.unregister()
        self.sub_wheels_cmd_executed.unregister()
        self.sub_actuator_limits.unregister()
        self.sub_switch.unregister()
        self.sub_fsm_mode.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)

        rospy.sleep(0.5)    #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)

    def publishCmd(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)
    def publishParamTunerCmd(self, param_tuner_msg):
        self.pub_param_tuner.publish(param_tuner_msg)
    def lqr(self, d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last):
        #Exception for dt_last=0
        if (dt_last==0) : dt_last=0.0001
        #Update state
        v_ref = self.lqr_gain*v_ref
        x = np.array([[d_est],[phi_est],[self.d_int]])
        #Adapt State Space to current time step
        a = np.array([[1., v_ref*dt_last, 0], [0, 1., 0], [-dt_last, -0.5*v_ref*dt_last*dt_last, 1]])
        b = np.array([[0.5*v_ref*dt_last*dt_last], [dt_last], [-v_ref*dt_last*dt_last*dt_last/6]])
        #Solve ricatti equation
        (x_ric, l, g) = control.dare(a, b, self.q, self.r)
        #feed trough velocity
        v = v_ref
        #Change sign on on K_I (because Ricatti equation returns [K_d, K_phi, -K_I])
        g[[0],[2]] = -g[[0],[2]]
        #Calculate new input
        omega = -g.dot(x)*self.lqr_gain
        #Update integral term
        self.d_int = self.d_int + d_est * dt_last
        return (v, omega)


    def cbPose(self, pose_msg):
        self.lane_reading = pose_msg

        # Calculating the delay image processing took
        timestamp_now = rospy.Time.now()
        dt = 0.0001
        image_delay_stamp = timestamp_now - self.lane_reading.header.stamp

        # delay from taking the image until now in seconds
        image_delay = image_delay_stamp.secs + image_delay_stamp.nsecs / 1e9

        prev_cross_track_err = self.cross_track_err
        #filter the heading error by a mean of the last 5 input signals
        self.pppp_heading_err = self.ppp_heading_err
        self.ppp_heading_err = self.prev_prev_heading_err
        self.prev_prev_heading_err = self.prev_heading_err
        self.prev_heading_err = self.heading_err

        self.cross_track_err = pose_msg.d - self.d_offset
        self.heading_err = pose_msg.phi

        filtered_heading_err = (self.heading_err+self.prev_heading_err+self.prev_prev_heading_err+self.ppp_heading_err+self.pppp_heading_err)/5

        v, omega = self.lqr(self.cross_track_err, self.heading_err, pose_msg.d_ref, pose_msg.phi_ref, pose_msg.v_ref, image_delay, dt)
        #print (v, omega)

        car_control_msg = Twist2DStamped()
        car_control_msg.header = pose_msg.header
        #car_control_msg.v = pose_msg.v_ref                                     #v for PI control
        car_control_msg.v = v                                              #v for LQRI

        if car_control_msg.v > self.actuator_limits.v:
            car_control_msg.v = self.actuator_limits.v

        if math.fabs(self.cross_track_err) > self.d_thres:
            rospy.logerr("inside threshold ")
            self.cross_track_err = self.cross_track_err / math.fabs(self.cross_track_err) * self.d_thres

        currentMillis = int(round(time.time() * 1000))

        if self.last_ms is not None:
            dt = (currentMillis - self.last_ms) / 1000.0
            self.cross_track_integral += self.cross_track_err * dt
            self.heading_integral += self.heading_err * dt

        if self.cross_track_integral > self.cross_track_integral_top_cutoff:
            self.cross_track_integral = self.cross_track_integral_top_cutoff
        if self.cross_track_integral < self.cross_track_integral_bottom_cutoff:
            self.cross_track_integral = self.cross_track_integral_bottom_cutoff

        if self.heading_integral > self.heading_integral_top_cutoff:
            self.heading_integral = self.heading_integral_top_cutoff
        if self.heading_integral < self.heading_integral_bottom_cutoff:
            self.heading_integral = self.heading_integral_bottom_cutoff

        if abs(self.cross_track_err) <= 0.011:  # TODO: replace '<= 0.011' by '< delta_d' (but delta_d might need to be sent by the lane_filter_node.py or even lane_filter.py)
            self.cross_track_integral = 0
        if abs(self.heading_err) <= 0.051:  # TODO: replace '<= 0.051' by '< delta_phi' (but delta_phi might need to be sent by the lane_filter_node.py or even lane_filter.py)
            self.heading_integral = 0
        if np.sign(self.cross_track_err) != np.sign(prev_cross_track_err):  # sign of error changed => error passed zero
            self.cross_track_integral = 0
        if np.sign(self.heading_err) != np.sign(self.prev_heading_err):  # sign of error changed => error passed zero
            self.heading_integral = 0
        if self.wheels_cmd_executed.vel_wheel == 0:  # if actual velocity sent to the motors is zero
            self.cross_track_integral = 0
            self.heading_integral = 0

        omega_feedforward = car_control_msg.v * pose_msg.curvature_ref
        if self.main_pose_source == "lane_filter" and not self.use_feedforward_part:
            omega_feedforward = 0

        # Scale the parameters linear such that their real value is at 0.22m/s TODO do this nice that  * (0.22/self.v_bar)
        #omega = self.k_d * (0.22/self.v_bar) * self.cross_track_err + self.k_theta * (0.22/self.v_bar) * self.heading_err + self.k_Dd * (0.22/self.v_bar) *(self.cross_track_err-prev_cross_track_err)/dt + self.k_Dphi * (0.22/self.v_bar) *(self.heading_err-self.prev_heading_err)/dt
        #omega += (omega_feedforward)


        #if not self.fsm_state == "SAFE_JOYSTICK_CONTROL":
        #    # apply integral correction (these should not affect radius, hence checked afterwards)
        #    omega -= self.k_Id * (0.22/self.v_bar) * self.cross_track_integral
        #    omega -= self.k_Iphi * (0.22/self.v_bar) * self.heading_integral

        if car_control_msg.v == 0:
            omega = 0
        #else:
        # check if velocity is large enough such that car can actually execute desired omega
        #    if car_control_msg.v - 0.5 * math.fabs(omega) * 0.1 < 0.065:
        #        car_control_msg.v = 0.065 + 0.5 * math.fabs(omega) * 0.1




        # apply magic conversion factors
        #car_control_msg.v = car_control_msg.v * self.velocity_to_m_per_s
        #car_control_msg.omega = omega * self.omega_to_rad_per_s

        #omega = car_control_msg.omega
        self.outputs.append(omega)
        del self.outputs[0]
        omega = sum(self.outputs) / len(self.outputs)
        if omega > self.omega_max: omega = self.omega_max
        if omega < self.omega_min: omega = self.omega_min
        #omega += self.omega_ff
        car_control_msg.omega = omega
        self.publishCmd(car_control_msg)
        param_tuner_help_msg = ParamTuner()
        param_tuner_help_msg.header = pose_msg.header
        param_tuner_help_msg.cross_track_err = self.cross_track_err
        param_tuner_help_msg.cross_track_integral = self.cross_track_integral
        param_tuner_help_msg.diff_cross_track_err = self.cross_track_err - prev_cross_track_err
        param_tuner_help_msg.heading_err = self.heading_err
        param_tuner_help_msg.heading_integral = self.heading_integral
        param_tuner_help_msg.diff_heading_err = self.heading_err - self.prev_heading_err
        param_tuner_help_msg.dt = dt
        self.publishParamTunerCmd(param_tuner_help_msg)
        self.last_ms = currentMillis



if __name__ == "__main__":

    rospy.init_node("lane_controller_node", anonymous=False)  # adapted to sonjas default file

    lane_control_node = lane_controller()
rospy.spin()