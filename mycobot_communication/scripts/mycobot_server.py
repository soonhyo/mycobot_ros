#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import copy
import os
import sys
import signal
import math
import threading

from pymycobot.mycobot import MyCobot

import rospy
from informatized_body_msgs.msg import Float32MultiArrayStamped
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion
import tf
import numpy as np


class MycobotServer():
    def __init__(self):
        rospy.init_node("mycobot_server")
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = rospy.get_param("~baud", 115200)
        rospy.loginfo("Connect mycobot on %s,%s" % (port, baud))
        self.mc = MyCobot(port, baud)
        self.lock = threading.Lock()

        self.real_angles = None
        self.joint_states_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        self.end_coords_pub = rospy.Publisher("end_coords", PoseStamped, queue_size=10)
        self.joint_command_sub = rospy.Subscriber("joint_command", Float32MultiArrayStamped, self.joint_command_callback)
        self.servo_command_sub = rospy.Subscriber("servo_command", String, self.servo_command_callback)

        self.hz = rospy.get_param("~hz", 20)
        self.rate = rospy.Rate(self.hz)

    def run(self):
        while not rospy.is_shutdown():

            current_stamp = rospy.Time.now()

            # joint angle publisher
            prev_real_angles = None
            if self.real_angles is not None:
                prev_real_angles = copy.deepcopy(self.real_angles)
            self.real_angles = self.mc.get_angles()
            # print(self.real_angles)
            if prev_real_angles is not None:
                if not self.real_angles:
                    self.real_angles = prev_real_angles
                    rospy.logwarn("cannot get real angles")
                elif np.linalg.norm(np.asarray(self.real_angles)-np.asarray(prev_real_angles)) > 60: # [deg]
                    self.real_angles = prev_real_angles
                    rospy.logwarn("real angles jumped")
            msg = JointState()
            msg.header.stamp = current_stamp
            for i, ang in enumerate(self.real_angles):
               msg.name.append('joint' + str(i+1))
               msg.position.append(ang / 180.0 * math.pi)
            self.joint_states_pub.publish(msg)

            # end coords publisher
            coords = self.mc.get_coords()
            if coords:
                msg = PoseStamped()
                msg.header.stamp = current_stamp
                msg.pose.position.x = coords[0]
                msg.pose.position.y = coords[1]
                msg.pose.position.z = coords[2]
                q = tf.transformations.quaternion_from_euler(coords[3], coords[4], coords[5])
                msg.pose.orientation.x = q[0]
                msg.pose.orientation.y = q[1]
                msg.pose.orientation.z = q[2]
                msg.pose.orientation.w = q[3]
                self.end_coords_pub.publish(msg)

            self.rate.sleep()

    # angle is rad, vel-factor has no dim
    def joint_command_callback(self, msg):
        ref_angles = list(msg.data[:len(self.real_angles)])
        ref_vel = int(msg.data[len(self.real_angles)])
        rospy.loginfo("{} : {}".format(ref_angles, ref_vel))
        self.mc.send_angles(ref_angles, ref_vel)

    def servo_command_callback(self, msg):
        if msg.data == "on":
            self.mc.send_angles(self.real_angles, 0)
            rospy.loginfo("servo on")
        elif msg.data == "off":
            self.mc.release_all_servos()
            rospy.loginfo("servo off")
        elif msg.data == "start-grasp":
            self.mc.set_gripper_state(1, 80)
            rospy.loginfo("start grasp")
        elif msg.data == "stop-grasp":
            self.mc.set_gripper_state(0, 80)
            rospy.loginfo("stop grasp")


def main():
    ms = MycobotServer()
    ms.run()

if __name__ == '__main__':
    main()
