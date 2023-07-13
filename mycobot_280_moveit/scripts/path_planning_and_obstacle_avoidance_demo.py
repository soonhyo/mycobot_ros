#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, roslib, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MoveItPlanningDemo:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("moveit_ik_demo")

        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)

        self.arm = moveit_commander.MoveGroupCommander("arm_group")

        self.end_effector_link = self.arm.get_end_effector_link()

        self.reference_frame = "link1"
        self.arm.set_pose_reference_frame(self.reference_frame)

        self.arm.allow_replanning(True)

        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)

    def moving(self):

        self.arm.set_named_target("init_pose")
        self.arm.go()
        rospy.sleep(2)

        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.132
        target_pose.pose.position.y = -0.150
        target_pose.pose.position.z = 0.075
        target_pose.pose.orientation.x = 0.026
        target_pose.pose.orientation.y = 1.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.014

        self.arm.set_start_state_to_current_state()

        self.arm.set_pose_target(target_pose, self.end_effector_link)

        plan_success, traj, planning_time, error_code = self.arm.plan()

        self.arm.execute(traj)
        rospy.sleep(1)

        self.arm.shift_pose_target(1, 0.12, self.end_effector_link)
        self.arm.go()
        rospy.sleep(1)

        self.arm.shift_pose_target(1, 0.1, self.end_effector_link)
        self.arm.go()
        rospy.sleep(1)

        # self.arm.shift_pose_target(3, -1.57, end_effector_link)
        # self.arm.go()
        # rospy.sleep(1)

    def run(self):
        self.scene.remove_world_object("suit")

        self.moving()

        quat = quaternion_from_euler(3.1415, 0, -1.57)

        suit_post = PoseStamped()
        suit_post.header.frame_id = self.reference_frame
        suit_post.pose.position.x = 0.0
        suit_post.pose.position.y = 0.0
        suit_post.pose.position.z = -0.02
        suit_post.pose.orientation.x = quat[0]
        suit_post.pose.orientation.y = quat[1]
        suit_post.pose.orientation.z = quat[2]
        suit_post.pose.orientation.w = quat[3]

        suit_path = (
            roslib.packages.get_pkg_dir("mycobot_description")
            + "/urdf/mycobot/suit_env.dae"
        )
        # need `pyassimp==3.3`
        self.scene.add_mesh("suit", suit_post, suit_path)
        rospy.sleep(2)

        self.moving()

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    o = MoveItPlanningDemo()
    o.run()
