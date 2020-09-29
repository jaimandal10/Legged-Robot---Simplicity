#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float64

class JointPub(object):
    def __init__(self):

        self.publishers_array = []

        self._shoulder_rf_joint_pub = rospy.Publisher('/simplicity/Rev14_position_controller/command', Float64, queue_size=1)
        self._shoulder_lf_joint_pub = rospy.Publisher('/simplicity/Rev15_position_controller/command', Float64, queue_size=1)
        self._shoulder_lb_joint_pub = rospy.Publisher('/simplicity/Rev16_position_controller/command', Float64, queue_size=1)
        self._shoulder_rb_joint_pub = rospy.Publisher('/simplicity/Rev17_position_controller/command', Float64, queue_size=1)

        self._thigh_lf_joint_pub = rospy.Publisher('/simplicity/Rev18_position_controller/command', Float64, queue_size=1)
        self._thigh_rf_joint_pub = rospy.Publisher('/simplicity/Rev19_position_controller/command', Float64, queue_size=1)
        self._thigh_rb_joint_pub = rospy.Publisher('/simplicity/Rev20_position_controller/command', Float64, queue_size=1)
        self._thigh_lb_joint_pub = rospy.Publisher('/simplicity/Rev21_position_controller/command', Float64, queue_size=1)

        self._leg_lf_joint_pub = rospy.Publisher('/simplicity/Rev35_position_controller/command', Float64, queue_size=1)
        self._leg_rf_joint_pub = rospy.Publisher('/simplicity/Rev36_position_controller/command', Float64, queue_size=1)        
        self._leg_lb_joint_pub = rospy.Publisher('/simplicity/Rev37_position_controller/command', Float64, queue_size=1)
        self._leg_rb_joint_pub = rospy.Publisher('/simplicity/Rev38_position_controller/command', Float64, queue_size=1)

        self.publishers_array.append(self._shoulder_rf_joint_pub)
        self.publishers_array.append(self._shoulder_lf_joint_pub)
        self.publishers_array.append(self._shoulder_lb_joint_pub)
        self.publishers_array.append(self._shoulder_rb_joint_pub)

        self.publishers_array.append(self._thigh_lf_joint_pub)
        self.publishers_array.append(self._thigh_rf_joint_pub)
        self.publishers_array.append(self._thigh_rb_joint_pub)
        self.publishers_array.append(self._thigh_lb_joint_pub)

        self.publishers_array.append(self._leg_lf_joint_pub)
        self.publishers_array.append(self._leg_rf_joint_pub)
        self.publishers_array.append(self._leg_lb_joint_pub)
        self.publishers_array.append(self._leg_rb_joint_pub)

        self.init_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def set_init_pose(self):
        """
        Sets joints to initial position [0,0,0]
        :return:
        """
        self.check_publishers_connection()
        self.move_joints(self.init_pos)


    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  

        # Shoulders
        while (self._shoulder_rf_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to shoulder_rf yet, so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("shoulder_rf Publisher Connected")

        while (self._shoulder_lf_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to shoulder_lf yet, so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("shoulder_lf Publisher Connected")

        while (self._shoulder_lb_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to shoulder_lb yet, so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("shoulder_lb Publisher Connected")

        while (self._shoulder_rb_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to shoulder_rb yet, so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("shoulder_rb Publisher Connected")

        # Thighs
        while (self._thigh_lf_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to thigh_lf yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("thigh_lf Publisher Connected")

        while (self._thigh_rf_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to thigh_rf yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("thigh_rf Publisher Connected")

        while (self._thigh_rb_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to thigh_rb yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("thigh_rb Publisher Connected")

        while (self._thigh_lb_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to thigh_lb yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("thigh_lb Publisher Connected")

        # Legs
        while (self._leg_lf_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to leg_lf yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("leg_lf Publisher Connected")

        while (self._leg_rf_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to leg_rf yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("leg_rf Publisher Connected")

        while (self._leg_lb_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to leg_lb yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("leg_lb Publisher Connected")

        while (self._leg_rb_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to leg_rb yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("leg_rb Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def joint_mono_des_callback(self, msg):
        rospy.logdebug(str(msg.joint_state.position))

        self.move_joints(msg.joint_state.position)

    def move_joints(self, joints_array):

        i = 0
        for publisher_object in self.publishers_array:
          joint_value = Float64()
          joint_value.data = joints_array[i]
          rospy.logdebug("JointsPos>>"+str(joint_value))
          publisher_object.publish(joint_value)
          i += 1


    def start_loop(self, rate_value = 2.0):
        rospy.logdebug("Start Loop")
        pos1 = [0.0,0.0,1.6]
        pos2 = [0.0,0.0,-1.6]
        position = "pos1"
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
          if position == "pos1":
            self.move_joints(pos1)
            position = "pos2"
          else:
            self.move_joints(pos2)
            position = "pos1"
          rate.sleep()

    def start_sinus_loop(self, rate_value = 2.0):
        rospy.logdebug("Start Loop")
        w = 0.0
        x = 2.0*math.sin(w)
        #pos_x = [0.0,0.0,x]
        #pos_x = [x, 0.0, 0.0]
        pos_x = [0.0, x, 0.0]
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            self.move_joints(pos_x)
            w += 0.05
            x = 2.0 * math.sin(w)
            #pos_x = [0.0, 0.0, x]
            #pos_x = [x, 0.0, 0.0]
            pos_x = [0.0, x, 0.0]
            rate.sleep()


if __name__=="__main__":
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()
    rate_value = 50.0
    #joint_publisher.start_loop(rate_value)
    # joint_publisher.start_sinus_loop(rate_value)
