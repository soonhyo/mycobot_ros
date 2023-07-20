#! /usr/bin/env python3
import rospy
import cv2
import actionlib
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

from mycobot_280_moveit.msg import *

class PathFollowClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient('path_follower', PathFollowAction)
        self.client.wait_for_server()
        self.test()

    def test(self):

        # Create an action goal and send it to the server
        pose = Pose()
        # pose.position.x = 0
        # pose.position.y = 0
        # pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0

        frame_id = "g_base"

        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = frame_id
        pose_array.poses = [pose]

        goal = PathFollowGoal()
        goal.path = pose_array

        self.client.send_goal(goal)

        # Wait for the result and process it
        self.client.wait_for_result()
        result = self.client.get_result()
        if self.client.get_state() == actionlib.GoalStatus.ABORTED:
            rospy.loginfo("Failed Moveit")
            return
        rospy.loginfo("Succeeded Moveit")

if __name__ == '__main__':
    rospy.init_node('path_follower_client')
    client = PathFollowClient()
    rospy.spin()
