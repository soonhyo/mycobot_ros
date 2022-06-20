#!/usr/bin/env python2

"""[summary]
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script.
Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyUSB0'
    baud: serial prot baudrate. Defaults is 115200.
"""

import rospy
from sensor_msgs.msg import JointState
from pymycobot.mybuddy import MyBuddy
import time
import os
import math

mb = None

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    # print(data.position)
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(value)
    

    print(data_list)

    data_list1 = data_list[:6]
    data_list2 = data_list[6:-1]
    data_list3 = data_list[-1:]

    print("left_arm: %s" % data_list1)
    print("right_arm: %s" % data_list2)
    print("waist: %s" % data_list3)

    print("\n")
    mb.send_radians(1,data_list1, 50)
    time.sleep(0.05)
    mb.send_radians(2,data_list2, 50)
    time.sleep(0.02)

    # print(data_list3[0])
    # mb.send_angle(3,1,data_list3[0]* (180 / math.pi),350)
    # mb.send_radians(3,data_list3, 50)

    mb.set_encoder(3,1,data_list3[0]*4096/(2*math.pi)+2048,1)
    time.sleep(0.02)

def listener():
    global mb
    rospy.init_node("control_slider", anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)
    port = rospy.get_param("~port", os.popen("ls /dev/ttyACM*").readline()[:-1])
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mb = MyBuddy(port, baud)

    # spin() simply keeps python from exiting until this node is stopped
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
