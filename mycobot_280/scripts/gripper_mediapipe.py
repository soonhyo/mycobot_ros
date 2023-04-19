#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from pymycobot.mycobot import MyCobot
import time

class GripperController:
    def __init__(self):
        rospy.init_node("gripper_controller")
        self.mc =MyCobot("/dev/mycobot", "115200")
        self.mc.set_encoders([2048, 2048, 2048, 2048, 2048, 2048], 20)
        rospy.sleep(3)

        rospy.Subscriber("/mediapipe_hands/angles", Float32MultiArray, self.angles_cb)
        rospy.spin()

    def angles_cb(self, msg):
        angles = msg.data
        if len(angles)!=0:
            value = int(angles[1]/3.2*(2048-1300))+1300 # for test, just use index angle
            self.mc.set_encoder(7, value)
            # rospy.sleep(0.1)
        # print(angles)


if __name__=="__main__":
    app=GripperController()


