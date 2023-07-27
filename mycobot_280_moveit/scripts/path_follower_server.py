#!/usr/bin/env python3
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from mycobot_280_moveit.msg import *
import tf
from geometry_msgs.msg import Pose, PoseArray
from tf.transformations import translation_matrix, quaternion_matrix, compose_matrix, euler_from_quaternion

class PathFollowServer(object):
    def __init__(self, group_name):
        rospy.init_node("path_follower_server", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        self.listener =tf.TransformListener()
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # group_name = "arm_group"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        #init action server
        self.action_server = actionlib.SimpleActionServer('path_follower', PathFollowAction, self.execute_cb, False)
        self.action_server.start()
        rospy.spin()

    def transform_pose_array(self, pose_array, target_frame, source_frame):
        """
        Transform a PoseArray from source_frame to target_frame using tf lookupTransform
        """
        # Initialize a TF listener
        listener = tf.TransformListener()

        # Wait for the transformation to become available
        listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))

        # Get the transform from source_frame to target_frame
        try:
            (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        except:
            return
        # Create a PoseArray to store the transformed poses
        transformed_pose_array = PoseArray()
        transformed_pose_array.header.frame_id = target_frame

        rospy.loginfo(trans)
        # Create a transformation matrix from the translation and rotation
        transform_matrix = compose_matrix(
            translate=trans,
            angles=euler_from_quaternion(rot)
        )

        # Iterate through each pose in the original PoseArray and transform it
        for pose in pose_array.poses:
            # Create a transformation matrix for the pose
            pose_matrix = compose_matrix(
                translate=(pose.position.x, pose.position.y, pose.position.z),
                angles=euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
            )

            # Combine the pose transformation with the frame transformation
            transformed_matrix = transform_matrix.dot(pose_matrix)

            # Extract the translation and rotation from the transformed matrix
            transformed_trans = tuple(transformed_matrix[:3, 3])
            transformed_quat = tuple(tf.transformations.quaternion_from_matrix(transformed_matrix))

            # Create a new Pose message from the transformed translation and rotation
            transformed_pose = Pose()
            transformed_pose.position.x, transformed_pose.position.y, transformed_pose.position.z = transformed_trans
            transformed_pose.orientation.x, transformed_pose.orientation.y, transformed_pose.orientation.z, transformed_pose.orientation.w = transformed_quat

            # Append the transformed pose to the transformed PoseArray
            transformed_pose_array.poses.append(transformed_pose)

        return transformed_pose_array
    def execute_cb(self, goal):
        frame_id = goal.path.header.frame_id
        succeed = False
        if not goal.path.poses:
            self.action_server.set_aborted()

        # Transform the PoseArray from frame1 to frame2
        transformed_pose_array = self.transform_pose_array(goal.path, 'g_base', frame_id)
        poses = transformed_pose_array.poses

        self.move_group.clear_pose_targets()

        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        waypoints.append(wpose)
        for pose in poses:
            wpose = pose
            # wpose.header.stamp = rospy.Time.now()
            # wpose.header.frame_id = frame_id
            waypoints.append(copy.deepcopy(wpose))
        rospy.loginfo(waypoints)

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

        succeed = self.move_group.execute(plan, wait=True)
        rospy.sleep(5)
        result_msg = PathFollowResult()
        result_msg.succeed = succeed

        if succeed:
            self.action_server.set_succeeded(result_msg)
        else:
            self.action_server.set_aborted(result_msg)

if __name__ == '__main__':
    server = PathFollowServer("arm_group")


# scale=1
# waypoints = []

# wpose = move_group.get_current_pose().pose
# wpose.position.z -= scale * 0.1  # First move up (z)
# wpose.position.y += scale * 0.2  # and sideways (y)
# waypoints.append(copy.deepcopy(wpose))

# wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
# waypoints.append(copy.deepcopy(wpose))

# wpose.position.y -= scale * 0.1  # Third move sideways (y)
# waypoints.append(copy.deepcopy(wpose))

# # We want the Cartesian path to be interpolated at a resolution of 1 cm
# # which is why we will specify 0.01 as the eef_step in Cartesian
# # translation.  We will disable the jump threshold by setting it to 0.0,
# # ignoring the check for infeasible jumps in joint space, which is sufficient
# # for this tutorial.
# (plan, fraction) = move_group.compute_cartesian_path(
#     waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
# )  # jump_threshold

# print(plan)


# display_trajectory = moveit_msgs.msg.DisplayTrajectory()
# display_trajectory.trajectory_start = robot.get_current_state()
# display_trajectory.trajectory.append(plan)
# # Publish
# display_trajectory_publisher.publish(display_trajectory)


# move_group.execute(plan, wait=True)