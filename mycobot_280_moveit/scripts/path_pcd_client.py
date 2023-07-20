#! /usr/bin/env python3
import rospy
import cv2
import actionlib
import message_filters
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from hair_flow_detect.msg import *
from mycobot_280_moveit.msg import *
import time
class PathPcdClient(object):
    def __init__(self):
        self.pathfollowclient = actionlib.SimpleActionClient('path_follower', PathFollowAction)
        self.pathfollowclient.wait_for_server()
        rospy.loginfo("path follow server online")

    def run(self, imgmsg, pcdmsg):
        ### path follower ###
        goal = PathFollowGoal()
        goal.path = pathpcd_result.pose_array

        self.pathfollowclient.send_goal(goal)

        # Wait for the result and process it
        self.pathfollowclient.wait_for_result()
        pathfollow_result = self.pathfollowclient.get_result()
        if self.pathfollowclient.get_state() == actionlib.GoalStatus.ABORTED:
            rospy.loginfo("Failed Moveit")
            return
        rospy.loginfo("Succeeded Moveit")

if __name__ == '__main__':
    rospy.init_node('path_follower_client')
    client = PathPcdClient()
    client.run()
    rospy.spin()
