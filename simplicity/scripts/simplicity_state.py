#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
import tf
import numpy
import math

"""
 wrenches:
      -
        force:
          x: -0.134995398774
          y: -0.252811705608
          z: -0.0861598399337
        torque:
          x: -0.00194729925705
          y: 0.028723398244
          z: -0.081229664152
    total_wrench:
      force:
        x: -0.134995398774
        y: -0.252811705608
        z: -0.0861598399337
      torque:
        x: -0.00194729925705
        y: 0.028723398244
        z: -0.081229664152
    contact_positions:
      -
        x: -0.0214808318267
        y: 0.00291348151391
        z: -0.000138379966267
    contact_normals:
      -
        x: 0.0
        y: 0.0
        z: 1.0
    depths: [0.000138379966266991]
  -
    info: "Debug:  i:(2/4)     my geom:monoped::lowerleg::lowerleg_contactsensor_link_collision_1\
  \   other geom:ground_plane::link::collision         time:50.405000000\n"
    collision1_name: "monoped::lowerleg::lowerleg_contactsensor_link_collision_1"
    collision2_name: "ground_plane::link::collision"

"""
"""
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
gazebo_msgs/ContactState[] states
  string info
  string collision1_name
  string collision2_name
  geometry_msgs/Wrench[] wrenches
    geometry_msgs/Vector3 force
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 torque
      float64 x
      float64 y
      float64 z
  geometry_msgs/Wrench total_wrench
    geometry_msgs/Vector3 force
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 torque
      float64 x
      float64 y
      float64 z
  geometry_msgs/Vector3[] contact_positions
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3[] contact_normals
    float64 x
    float64 y
    float64 z
  float64[] depths
"""

class SimplicityState(object):

    def __init__(self, max_height, min_height, abs_max_roll, abs_max_pitch, joint_increment_value = 0.05, done_reward = -1000.0, alive_reward=10.0, desired_force=7.08, desired_yaw=0.0, weight_r1=1.0, weight_r2=1.0, weight_r3=1.0, weight_r4=1.0, weight_r5=1.0, discrete_division=10):
        rospy.logdebug("Starting SimplicityState Class object...")
        self.desired_world_point = Vector3(0.0, 0.0, 0.0)
        self._min_height = min_height
        self._max_height = max_height
        self._abs_max_roll = abs_max_roll
        self._abs_max_pitch = abs_max_pitch
        self._joint_increment_value = joint_increment_value
        self._done_reward = done_reward
        self._alive_reward = alive_reward
        self._desired_force = desired_force
        self._desired_yaw = desired_yaw

        self._weight_r1 = weight_r1
        self._weight_r2 = weight_r2
        self._weight_r3 = weight_r3
        self._weight_r4 = weight_r4
        self._weight_r5 = weight_r5

        self._list_of_observations = ["distance_from_desired_point",
                 "base_roll",
                 "base_pitch",
                 "base_yaw",
                 "contact_force_lf",
                 "contact_force_rf",
                 "contact_force_lb",
                 "contact_force_rb",
                 "joint_states_shoulder_rf",
                 "joint_states_shoulder_lf",
                 "joint_states_shoulder_lb",
                 "joint_states_shoulder_rb",
                 "joint_states_thigh_lf",
                 "joint_states_thigh_rf",
                 "joint_states_thigh_rb",
                 "joint_states_thigh_lb",
                 "joint_states_leg_lf",
                 "joint_states_leg_rf",
                 "joint_states_leg_lb",
                 "joint_states_leg_rb"
                 ]

        self._discrete_division = discrete_division
        # We init the observation ranges and We create the bins now for all the observations
        self.init_bins()

        self.base_position = Point()
        self.base_orientation = Quaternion()
        self.base_linear_acceleration = Vector3()
        self.contact_force_lf = Vector3()
        self.contact_force_rf = Vector3()
        self.contact_force_lb = Vector3()
        self.contact_force_rb = Vector3()
        self.joints_state = JointState()

        # Odom we only use it for the height detection and planar position ,
        # because in real robots this data is not trivial.
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # We use the IMU for orientation and linearacceleration detection
        rospy.Subscriber("/simplicity/imu/data", Imu, self.imu_callback)
        # We use it to get the contact force, to know if its in the air or stumping too hard.
        rospy.Subscriber("/back_left_contact_sensor_state", ContactsState, self.contact_callback_lb)
        rospy.Subscriber("/back_right_contact_sensor_state", ContactsState, self.contact_callback_rb)
        rospy.Subscriber("/front_left_contact_sensor_state", ContactsState, self.contact_callback_lf)
        rospy.Subscriber("/front_right_contact_sensor_state", ContactsState, self.contact_callback_rf)
        # We use it to get the joints positions and calculate the reward associated to it
        rospy.Subscriber("/simplicity/joint_states", JointState, self.joints_state_callback)

    def check_all_systems_ready(self):
        """
        We check that all systems are ready
        :return:
        """
        data_pose = None
        while data_pose is None and not rospy.is_shutdown():
            try:
                data_pose = rospy.wait_for_message("/odom", Odometry, timeout=0.1)
                self.base_position = data_pose.pose.pose.position
                rospy.logdebug("Current odom READY")
            except:
                rospy.logdebug("Current odom pose not ready yet, retrying for getting robot base_position")

        imu_data = None
        while imu_data is None and not rospy.is_shutdown():
            try:
                imu_data = rospy.wait_for_message("/simplicity/imu/data", Imu, timeout=0.1)
                self.base_orientation = imu_data.orientation
                self.base_linear_acceleration = imu_data.linear_acceleration
                rospy.logdebug("Current imu_data READY")
            except:
                rospy.logdebug("Current imu_data not ready yet, retrying for getting robot base_orientation, and base_linear_acceleration")

        contacts_data_lf = None
        while contacts_data_lf is None and not rospy.is_shutdown():
            try:
                contacts_data_lf = rospy.wait_for_message("/front_left_contact_sensor_state", ContactsState, timeout=0.1)
                for state in contacts_data_lf.states:
                    self.contact_force_lf = state.total_wrench.force
                rospy.logdebug("Current contacts_data_lf READY")
            except:
                rospy.logdebug("Current contacts_data_lf not ready yet, retrying")

        contacts_data_rf = None
        while contacts_data_rf is None and not rospy.is_shutdown():
            try:
                contacts_data_rf = rospy.wait_for_message("/front_right_contact_sensor_state", ContactsState, timeout=0.1)
                for state in contacts_data_rf.states:
                    self.contact_force_rf = state.total_wrench.force
                rospy.logdebug("Current contacts_data_rf READY")
            except:
                rospy.logdebug("Current contacts_data_rf not ready yet, retrying")

        contacts_data_lb = None
        while contacts_data_lb is None and not rospy.is_shutdown():
            try:
                contacts_data_lb = rospy.wait_for_message("/back_left_contact_sensor_state", ContactsState, timeout=0.1)
                for state in contacts_data_lb.states:
                    self.contact_force_lb = state.total_wrench.force
                rospy.logdebug("Current contacts_data_lb READY")
            except:
                rospy.logdebug("Current contacts_data_lb not ready yet, retrying")

        contacts_data_rb = None
        while contacts_data_rb is None and not rospy.is_shutdown():
            try:
                contacts_data_rb = rospy.wait_for_message("/back_right_contact_sensor_state", ContactsState, timeout=0.1)
                for state in contacts_data_rb.states:
                    self.contact_force_rb = state.total_wrench.force
                rospy.logdebug("Current contacts_data_rb READY")
            except:
                rospy.logdebug("Current contacts_data_rb not ready yet, retrying")

        joint_states_msg = None
        while joint_states_msg is None and not rospy.is_shutdown():
            try:
                joint_states_msg = rospy.wait_for_message("/simplicity/joint_states", JointState, timeout=0.1)
                self.joints_state = joint_states_msg
                rospy.logdebug("Current joint_states READY")
            except Exception as e:
                rospy.logdebug("Current joint_states not ready yet, retrying==>"+str(e))

        rospy.logdebug("ALL SYSTEMS READY")

    def set_desired_world_point(self, x, y, z):
        """
        Point where you want the Monoped to be
        :return:
        """
        self.desired_world_point.x = x
        self.desired_world_point.y = y
        self.desired_world_point.z = z


    def get_base_height(self):
        return self.base_position.z

    def get_base_rpy(self):
        euler_rpy = Vector3()
        euler = tf.transformations.euler_from_quaternion(
            [self.base_orientation.x, self.base_orientation.y, self.base_orientation.z, self.base_orientation.w])

        euler_rpy.x = euler[0]
        euler_rpy.y = euler[1]
        euler_rpy.z = euler[2]
        return euler_rpy

    def get_distance_from_point(self, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((self.base_position.x, self.base_position.y, self.base_position.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))

        distance = numpy.linalg.norm(a - b)

        return distance

    def get_contact_force_lf_magnitude(self):
        """
        You will see that because the X axis is the one pointing downwards, it will be the one with
        higher value when touching the floor
        For a Robot of total mas of 0.55Kg, a gravity of 9.81 m/sec**2, Weight = 0.55*9.81=5.39 N
        Falling from around 5centimetres ( negligible height ), we register peaks around
        Fx = 7.08 N
        :return:
        """
        contact_force_lf = self.contact_force_lf
        contact_force_lf_np = numpy.array((contact_force_lf.x, contact_force_lf.y, contact_force_lf.z))
        force_magnitude_lf = numpy.linalg.norm(contact_force_lf_np)

        return force_magnitude_lf

    def get_contact_force_rf_magnitude(self):
        contact_force_rf = self.contact_force_rf
        contact_force_rf_np = numpy.array((contact_force_rf.x, contact_force_rf.y, contact_force_rf.z))
        force_magnitude_rf = numpy.linalg.norm(contact_force_rf_np)

        return force_magnitude_rf
    
    def get_contact_force_lb_magnitude(self):
        contact_force_lb = self.contact_force_lb
        contact_force_lb_np = numpy.array((contact_force_lb.x, contact_force_lb.y, contact_force_lb.z))
        force_magnitude_lb = numpy.linalg.norm(contact_force_lb_np)

        return force_magnitude_lb

    def get_contact_force_rb_magnitude(self):
        contact_force_rb = self.contact_force_rb
        contact_force_rb_np = numpy.array((contact_force_rb.x, contact_force_rb.y, contact_force_rb.z))
        force_magnitude_rb = numpy.linalg.norm(contact_force_rb_np)

        return force_magnitude_rb

    def get_joint_states(self):
        return self.joints_state

    def odom_callback(self,msg):
        self.base_position = msg.pose.pose.position

    def imu_callback(self,msg):
        self.base_orientation = msg.orientation
        self.base_linear_acceleration = msg.linear_acceleration

    def contact_callback_lf(self,msg):
        """
        /lowerleg_contactsensor_state/states[0]/contact_positions ==> PointContact in World
        /lowerleg_contactsensor_state/states[0]/contact_normals ==> NormalContact in World

        ==> One is an array of all the forces, the other total,
         and are relative to the contact link referred to in the sensor.
        /lowerleg_contactsensor_state/states[0]/wrenches[]
        /lowerleg_contactsensor_state/states[0]/total_wrench
        :param msg:
        :return:
        """
        for state in msg.states:
            self.contact_force_lf = state.total_wrench.force

    def contact_callback_rf(self,msg):
        for state in msg.states:
            self.contact_force_rf = state.total_wrench.force

    def contact_callback_lb(self,msg):
        for state in msg.states:
            self.contact_force_lb = state.total_wrench.force

    def contact_callback_rb(self,msg):
        for state in msg.states:
            self.contact_force_rb = state.total_wrench.force   

    def joints_state_callback(self,msg):
        self.joints_state = msg

    def monoped_height_ok(self):

        height_ok = self._min_height <= self.get_base_height() < self._max_height
        return height_ok

    def monoped_orientation_ok(self):

        orientation_rpy = self.get_base_rpy()
        roll_ok = self._abs_max_roll > abs(orientation_rpy.x)
        pitch_ok = self._abs_max_pitch > abs(orientation_rpy.y)
        orientation_ok = roll_ok and pitch_ok
        return orientation_ok

    def calculate_reward_joint_position(self, weight=1.0):
        """
        We calculate reward base on the joints configuration. The more near 0 the better.
        :return:
        """
        acumulated_joint_pos = 0.0
        for joint_pos in self.joints_state.position:
            # Abs to remove sign influence, it doesnt matter the direction of turn.
            acumulated_joint_pos += abs(joint_pos)
            rospy.logdebug("calculate_reward_joint_position>>acumulated_joint_pos=" + str(acumulated_joint_pos))
        reward = weight * acumulated_joint_pos
        rospy.logdebug("calculate_reward_joint_position>>reward=" + str(reward))
        return reward

    def calculate_reward_joint_effort(self, weight=1.0):
        """
        We calculate reward base on the joints effort readings. The more near 0 the better.
        :return:
        """
        acumulated_joint_effort = 0.0
        for joint_effort in self.joints_state.effort:
            # Abs to remove sign influence, it doesnt matter the direction of the effort.
            acumulated_joint_effort += abs(joint_effort)
            rospy.logdebug("calculate_reward_joint_effort>>joint_effort=" + str(joint_effort))
            rospy.logdebug("calculate_reward_joint_effort>>acumulated_joint_effort=" + str(acumulated_joint_effort))
        reward = weight * acumulated_joint_effort
        rospy.logdebug("calculate_reward_joint_effort>>reward=" + str(reward))
        return reward

    def calculate_reward_contact_force(self, weight=1.0):
        """
        We calculate reward base on the contact force.
        The nearest to the desired contact force the better.
        We use exponential to magnify big departures from the desired force.
        Default ( 7.08 N ) desired force was taken from reading of the robot touching
        the ground from a negligible height of 5cm.
        :return:
        """
        force_magnitude_lf = self.get_contact_force_lf_magnitude()
        force_magnitude_rf = self.get_contact_force_rf_magnitude()
        force_magnitude_lb = self.get_contact_force_lb_magnitude()
        force_magnitude_rb = self.get_contact_force_rb_magnitude()
        force_displacement = force_magnitude_lf + force_magnitude_rf + force_magnitude_lb + force_magnitude_rb - self._desired_force

        rospy.logdebug("calculate_reward_contact_force>>force_magnitude_lf=" + str(force_magnitude_lf))
        rospy.logdebug("calculate_reward_contact_force>>force_magnitude_rf=" + str(force_magnitude_rf))
        rospy.logdebug("calculate_reward_contact_force>>force_magnitude_lb=" + str(force_magnitude_lb))
        rospy.logdebug("calculate_reward_contact_force>>force_magnitude_rb=" + str(force_magnitude_rb))

        rospy.logdebug("calculate_reward_contact_force>>force_displacement=" + str(force_displacement))
        # Abs to remove sign
        reward = weight * abs(force_displacement)
        rospy.logdebug("calculate_reward_contact_force>>reward=" + str(reward))
        return reward

    def calculate_reward_orientation(self, weight=1.0):
        """
        We calculate the reward based on the orientation.
        The more its closser to 0 the better because it means its upright
        desired_yaw is the yaw that we want it to be.
        to praise it to have a certain orientation, here is where to set it.
        :return:
        """
        curren_orientation = self.get_base_rpy()
        yaw_displacement = curren_orientation.z - self._desired_yaw
        rospy.logdebug("calculate_reward_orientation>>[R,P,Y]=" + str(curren_orientation))
        acumulated_orientation_displacement = abs(curren_orientation.x) + abs(curren_orientation.y) + abs(yaw_displacement)
        reward = weight * acumulated_orientation_displacement
        rospy.logdebug("calculate_reward_orientation>>reward=" + str(reward))
        return reward

    def calculate_reward_distance_from_des_point(self, weight=1.0):
        """
        We calculate the distance from the desired point.
        The closser the better
        :param weight:
        :return:reward
        """
        distance = self.get_distance_from_point(self.desired_world_point)
        reward = weight * distance
        rospy.logdebug("calculate_reward_orientation>>reward=" + str(reward))
        return reward

    def calculate_total_reward(self):
        """
        We consider VERY BAD REWARD -7 or less
        Perfect reward is 0.0, and total reward 1.0.
        The defaults values are chosen so that when the robot has fallen or very extreme joint config:
        r1 = -8.04
        r2 = -8.84
        r3 = -7.08
        r4 = -10.0 ==> We give priority to this, giving it higher value.
        :return:
        """

        r1 = self.calculate_reward_joint_position(self._weight_r1)
        r2 = self.calculate_reward_joint_effort(self._weight_r2)
        # Desired Force in Newtons, taken form idle contact with 9.81 gravity.
        r3 = self.calculate_reward_contact_force(self._weight_r3)
        r4 = self.calculate_reward_orientation(self._weight_r4)
        r5 = self.calculate_reward_distance_from_des_point(self._weight_r5)

        # The sign depend on its function.
        total_reward = self._alive_reward - r1 - r2 - r3 - r4 - r5

        rospy.logdebug("###############")
        rospy.logdebug("alive_bonus=" + str(self._alive_reward))
        rospy.logdebug("r1 joint_position=" + str(r1))
        rospy.logdebug("r2 joint_effort=" + str(r2))
        rospy.logdebug("r3 contact_force=" + str(r3))
        rospy.logdebug("r4 orientation=" + str(r4))
        rospy.logdebug("r5 distance=" + str(r5))
        rospy.logdebug("total_reward=" + str(total_reward))
        rospy.logdebug("###############")

        return total_reward




    def get_observations(self):
        """
        Returns the state of the robot needed for OpenAI QLearn Algorithm
        The state will be defined by an array of the:
        1) distance from desired point in meters
        2) The pitch orientation in radians
        3) the Roll orientation in radians
        4) the Yaw orientation in radians
        5) Force in contact sensor in Newtons
        6-7-8) State of the 3 joints in radians
        """

        distance_from_desired_point = self.get_distance_from_point(self.desired_world_point)

        base_orientation = self.get_base_rpy()
        base_roll = base_orientation.x
        base_pitch = base_orientation.y
        base_yaw = base_orientation.z

        contact_force_lf = self.get_contact_force_lf_magnitude()
        contact_force_rf = self.get_contact_force_rf_magnitude()
        contact_force_lb = self.get_contact_force_lb_magnitude()
        contact_force_rb = self.get_contact_force_rb_magnitude()

        joint_states = self.get_joint_states()

        joint_states_shoulder_rf = joint_states.position[0]
        joint_states_shoulder_lf = joint_states.position[1]
        joint_states_shoulder_lb = joint_states.position[2]
        joint_states_shoulder_rb = joint_states.position[3]

        joint_states_thigh_lf = joint_states.position[4]
        joint_states_thigh_rf = joint_states.position[5]
        joint_states_thigh_rb = joint_states.position[6]
        joint_states_thigh_lb = joint_states.position[7]

        joint_states_leg_lf = joint_states.position[8]
        joint_states_leg_rf = joint_states.position[9]
        joint_states_leg_lb = joint_states.position[10]
        joint_states_leg_rb = joint_states.position[11]

        observation = []
        for obs_name in self._list_of_observations:
            if obs_name == "distance_from_desired_point":
                observation.append(distance_from_desired_point)
            elif obs_name == "base_roll":
                observation.append(base_roll)
            elif obs_name == "base_pitch":
                observation.append(base_pitch)
            elif obs_name == "base_yaw":
                observation.append(base_yaw)

            elif obs_name == "contact_force_lf":
                observation.append(contact_force_lf)
            elif obs_name == "contact_force_rf":
                observation.append(contact_force_rf)
            elif obs_name == "contact_force_lb":
                observation.append(contact_force_lb)
            elif obs_name == "contact_force_rb":
                observation.append(contact_force_rb)

            elif obs_name == "joint_states_shoulder_lf":
                observation.append(joint_states_shoulder_lf)
            elif obs_name == "joint_states_shoulder_rf":
                observation.append(joint_states_shoulder_rf)
            elif obs_name == "joint_states_shoulder_lb":
                observation.append(joint_states_shoulder_lb)
            elif obs_name == "joint_states_shoulder_rb":
                observation.append(joint_states_shoulder_rb)

            elif obs_name == "joint_states_thigh_lf":
                observation.append(joint_states_thigh_lf)
            elif obs_name == "joint_states_thigh_rf":
                observation.append(joint_states_thigh_rf)
            elif obs_name == "joint_states_thigh_rb":
                observation.append(joint_states_thigh_rb)
            elif obs_name == "joint_states_thigh_lb":
                observation.append(joint_states_thigh_lb)

            elif obs_name == "joint_states_leg_lf":
                observation.append(joint_states_leg_lf)
            elif obs_name == "joint_states_leg_rf":
                observation.append(joint_states_leg_rf)
            elif obs_name == "joint_states_leg_lb":
                observation.append(joint_states_leg_lb)
            elif obs_name == "joint_states_leg_rb":
                observation.append(joint_states_leg_rb)            

            else:
                raise NameError('Observation Asked does not exist=='+str(obs_name))

        return observation

    def get_state_as_string(self, observation):
        """
        This function will do two things:
        1) It will make discrete the observations
        2) Will convert the discrete observations in to state tags strings
        :param observation:
        :return: state
        """
        observations_discrete = self.assign_bins(observation)
        string_state = ''.join(map(str, observations_discrete))
        return string_state

    def assign_bins(self, observation):
        """
        Will make observations discrete by placing each value into its corresponding bin
        :param observation:
        :return:
        """
        state_discrete = numpy.zeros(len(self._list_of_observations))
        for i in range(len(self._list_of_observations)):
            state_discrete[i] = numpy.digitize(observation[i], self._bins[i])
        return state_discrete

    def init_bins(self):
        """
        We initalise all related to the bins
        :return:
        """
        self.fill_observations_ranges()
        self.create_bins()

    def fill_observations_ranges(self):
        """
        We create the dictionary for the ranges of the data related to each observation
        :return:
        """
        self._obs_range_dict = {}
        for obs_name in self._list_of_observations:

            if obs_name == "distance_from_desired_point":
                # We consider the range as based on the range of distance allowed in height
                delta = self._max_height - self._min_height
                max_value = delta
                min_value = -delta
            elif obs_name == "base_roll":
                max_value = self._abs_max_roll
                min_value = -self._abs_max_roll
            elif obs_name == "base_pitch":
                max_value = self._abs_max_pitch
                min_value = -self._abs_max_pitch
            elif obs_name == "base_yaw":
                # We consider that 360 degrees is max range
                max_value = 2*math.pi
                min_value = -2*math.pi

            elif obs_name == "contact_force_lf" or obs_name == "contact_force_rf" or obs_name == "contact_force_lb" or obs_name == "contact_force_rb":
                # We consider that no force is the minimum, and the maximum is .5 times the desired_force/weight
                # We dont want to make a very big range because we might loose the desired force
                # in the middle.
                max_value = .5*self._desired_force
                min_value = 0.0

            # Max-min values from urdf
            elif obs_name == "joint_states_shoulder_lf":                
                max_value = 1.6
                min_value = -1.6
            elif obs_name == "joint_states_shoulder_rf":                
                max_value = 1.6
                min_value = -1.6
            elif obs_name == "joint_states_shoulder_lb":                
                max_value = 1.6
                min_value = -1.6
            elif obs_name == "joint_states_shoulder_rb":
                max_value = 1.6
                min_value = -1.6

            elif obs_name == "joint_states_thigh_lf":
                max_value = 1.6
                min_value = -1.6
            elif obs_name == "joint_states_thigh_rf":
                max_value = 1.6
                min_value = -1.6
            elif obs_name == "joint_states_thigh_lb":
                max_value = 1.6
                min_value = -1.6
            elif obs_name == "joint_states_thigh_rb":
                max_value = 1.6
                min_value = -1.6

            elif obs_name == "joint_states_leg_lf":
                max_value = 1.6
                min_value = -1.6
            elif obs_name == "joint_states_leg_rf":
                max_value = 1.6
                min_value = -1.6
            elif obs_name == "joint_states_leg_lb":
                max_value = 1.6
                min_value = -1.6
            elif obs_name == "joint_states_leg_rb":
                max_value = 1.6
                min_value = -1.6
                
            else:
                raise NameError('Observation Asked does not exist=='+str(obs_name))

            self._obs_range_dict[obs_name] = [min_value,max_value]

    def create_bins(self):
        """
        We create the Bins for the discretization of the observations
        self.desired_world_point = Vector3(0.0, 0.0, 0.0)
        self._min_height = min_height
        self._max_height = max_height
        self._abs_max_roll = abs_max_roll
        self._abs_max_pitch = abs_max_pitch
        self._joint_increment_value = joint_increment_value
        self._done_reward = done_reward
        self._alive_reward = alive_reward
        self._desired_force = desired_force
        self._desired_yaw = desired_yaw


        :return:bins
        """

        number_of_observations = len(self._list_of_observations)
        parts_we_disrcetize = self._discrete_division

        self._bins = numpy.zeros((number_of_observations, parts_we_disrcetize))
        for counter in range(number_of_observations):
            obs_name = self._list_of_observations[counter]
            min_value = self._obs_range_dict[obs_name][0]
            max_value = self._obs_range_dict[obs_name][1]
            self._bins[counter] = numpy.linspace(min_value, max_value, parts_we_disrcetize)


    def get_action_to_position(self, action):
        """
        Here we have the ACtions number to real joint movement correspondance.
        :param action: Integer that goes from 0 to 5, because we have 6 actions.
        :return:
        """
        # We get current Joints values
        joint_states = self.get_joint_states()
        joint_states_position = joint_states.position

        action_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        rospy.logdebug("get_action_to_position>>>"+str(joint_states_position))
        if action == 0: 
            action_position[0] = joint_states_position[0] + self._joint_increment_value
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3]
            action_position[4] = joint_states_position[4]
            action_position[5] = joint_states_position[5]
            action_position[6] = joint_states_position[6]
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]
            
        elif action == 1:
            action_position[0] = joint_states_position[0] - self._joint_increment_value
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3]
            action_position[4] = joint_states_position[4]
            action_position[5] = joint_states_position[5]
            action_position[6] = joint_states_position[6]
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]

        elif action == 2: 
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1] + self._joint_increment_value
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3]
            action_position[4] = joint_states_position[4]
            action_position[5] = joint_states_position[5]
            action_position[6] = joint_states_position[6]
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]

        elif action == 3:
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1] - self._joint_increment_value
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3]
            action_position[4] = joint_states_position[4]
            action_position[5] = joint_states_position[5]
            action_position[6] = joint_states_position[6]
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]

        elif action == 4: 
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2] + self._joint_increment_value
            action_position[3] = joint_states_position[3]
            action_position[4] = joint_states_position[4]
            action_position[5] = joint_states_position[5]
            action_position[6] = joint_states_position[6]
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]

        elif action == 5:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2] - self._joint_increment_value
            action_position[3] = joint_states_position[3]
            action_position[4] = joint_states_position[4]
            action_position[5] = joint_states_position[5]
            action_position[6] = joint_states_position[6]
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]
        
        elif action == 6: 
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] + self._joint_increment_value
            action_position[4] = joint_states_position[4]
            action_position[5] = joint_states_position[5]
            action_position[6] = joint_states_position[6]
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]
        
        elif action == 7:
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] - self._joint_increment_value
            action_position[4] = joint_states_position[4]
            action_position[5] = joint_states_position[5]
            action_position[6] = joint_states_position[6]
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]

        elif action == 8:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] + self._joint_increment_value
            action_position[5] = joint_states_position[5]
            action_position[6] = joint_states_position[6]
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]
        
        elif action == 9:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] - self._joint_increment_value
            action_position[5] = joint_states_position[5]
            action_position[6] = joint_states_position[6]
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]  

        elif action == 10:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] + self._joint_increment_value
            action_position[6] = joint_states_position[6]
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]

        elif action == 11:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] - self._joint_increment_value
            action_position[6] = joint_states_position[6]
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]  

        elif action == 12:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] 
            action_position[6] = joint_states_position[6] + self._joint_increment_value
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]

        elif action == 13:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] 
            action_position[6] = joint_states_position[6] - self._joint_increment_value
            action_position[7] = joint_states_position[7]
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]  

        elif action == 14:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] 
            action_position[6] = joint_states_position[6] 
            action_position[7] = joint_states_position[7] + self._joint_increment_value
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]

        elif action == 15:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] 
            action_position[6] = joint_states_position[6] 
            action_position[7] = joint_states_position[7] - self._joint_increment_value
            action_position[8] = joint_states_position[8]
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]  

        elif action == 16:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] 
            action_position[6] = joint_states_position[6] 
            action_position[7] = joint_states_position[7] 
            action_position[8] = joint_states_position[8] + self._joint_increment_value
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]

        elif action == 17:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] 
            action_position[6] = joint_states_position[6] 
            action_position[7] = joint_states_position[7] 
            action_position[8] = joint_states_position[8] - self._joint_increment_value
            action_position[9] = joint_states_position[9]
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]

        elif action == 18:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] 
            action_position[6] = joint_states_position[6] 
            action_position[7] = joint_states_position[7] 
            action_position[8] = joint_states_position[8] 
            action_position[9] = joint_states_position[9] + self._joint_increment_value
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]
        
        elif action == 19:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] 
            action_position[6] = joint_states_position[6] 
            action_position[7] = joint_states_position[7] 
            action_position[8] = joint_states_position[8] 
            action_position[9] = joint_states_position[9] - self._joint_increment_value
            action_position[10] = joint_states_position[10]
            action_position[11] = joint_states_position[11]

        elif action == 20:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] 
            action_position[6] = joint_states_position[6] 
            action_position[7] = joint_states_position[7] 
            action_position[8] = joint_states_position[8] 
            action_position[9] = joint_states_position[9] 
            action_position[10] = joint_states_position[10] + self._joint_increment_value
            action_position[11] = joint_states_position[11]

        elif action == 21:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] 
            action_position[6] = joint_states_position[6] 
            action_position[7] = joint_states_position[7] 
            action_position[8] = joint_states_position[8] 
            action_position[9] = joint_states_position[9] 
            action_position[10] = joint_states_position[10] - self._joint_increment_value
            action_position[11] = joint_states_position[11]

        elif action == 22:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] 
            action_position[6] = joint_states_position[6] 
            action_position[7] = joint_states_position[7] 
            action_position[8] = joint_states_position[8] 
            action_position[9] = joint_states_position[9] 
            action_position[10] = joint_states_position[10] 
            action_position[11] = joint_states_position[11] + self._joint_increment_value

        elif action == 23:  
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
            action_position[3] = joint_states_position[3] 
            action_position[4] = joint_states_position[4] 
            action_position[5] = joint_states_position[5] 
            action_position[6] = joint_states_position[6] 
            action_position[7] = joint_states_position[7] 
            action_position[8] = joint_states_position[8] 
            action_position[9] = joint_states_position[9] 
            action_position[10] = joint_states_position[10] 
            action_position[11] = joint_states_position[11] - self._joint_increment_value       

        return action_position

    def process_data(self):
        """
        We return the total reward based on the state in which we are in and if its done or not
        ( it fell basically )
        :return: reward, done
        """
        monoped_height_ok = self.monoped_height_ok()
        monoped_orientation_ok = self.monoped_orientation_ok()

        done = not(monoped_height_ok and monoped_orientation_ok)
        if done:
            rospy.logdebug("It fell, so the reward has to be very low")
            total_reward = self._done_reward
        else:
            rospy.logdebug("Calculate normal reward because it didn't fall.")
            total_reward = self.calculate_total_reward()

        return total_reward, done

    def testing_loop(self):

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.calculate_total_reward()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('monoped_state_node', anonymous=True)
    monoped_state = MonopedState(max_height=3.0,
                                 min_height=0.6,
                                 abs_max_roll=0.7,
                                 abs_max_pitch=0.7)
    monoped_state.testing_loop()
