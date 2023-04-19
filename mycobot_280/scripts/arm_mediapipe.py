#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import sys
from pynput import keyboard
from pymycobot.mycobot import MyCobot
import time
import readchar
class GripperController:
    def __init__(self):
        rospy.init_node("gripper_arm_controller")
        self.mc =MyCobot("/dev/mycobot", "115200")
        self.mc.set_encoders([2048, 2048, 2048, 2048, 2048, 2048], 20)
        self.start_position = []
        rospy.sleep(3)

        rospy.Subscriber("/mediapipe_hands/angles", Float32MultiArray, self.angles_cb)
        rospy.Subscriber("/mediapipe_hands/pose", PoseStamped, self.pose_cb)

        rospy.spin()

    def angles_cb(self, msg):
        angles = msg.data
        if len(angles)!=0:
            value = int(angles[1]/3.2*(2048-1300))+1300 # for test, just use index angle
            self.mc.set_encoder(7, value)
            # rospy.sleep(0.1)
        # print(angles)

    def pose_cb(self, msg):
        pose = msg.pose
        position = pose.position
        orientation = pose.orientation

        kb = readchar.readkey()
        if kb =='s':
            print(position)
            self.start_position = position

        if self.start_position:
            coords =self.mc.get_coords()
            print("coords:",coords)

            if coords:
                coords[0] += self.start_position.x - position.x
                coords[1] += self.start_position.y - position.y
                coords[2] += self.start_position.z - position.z

                self.mc.send_coords(coords, 5, 1)

        # if len(angles)!=0:
        #     self.mc.set_encoders([], value)
            # rospy.sleep(0.1)
        # print(angles)


if __name__=="__main__":
    try:
        app=GripperController()
    except:
        exit(0)

