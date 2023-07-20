from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
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

class FollowTrajectory(object):
    def __init__(self, group_name):
        rospy.init_node("TODO", anonymous=True)
        self.moveit_commander.roscpp_initialize(sys.argv)

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
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        #init action server
        self.action_server = actionlib.SimpleActionServer('path_follower', PathFollowAction, self.execute_cb, False)
        self.action_server.start()

    def execute_cb(self, goal):
        poses= goal.pose_array.poses
        frame_id = goal.pose_array.header.frame_id
        succeed = False
        if poses:
            succeed = self.follow_trajectory_cb(poses)
        else:
            self.action_server.set_aborted()

        waypoints = []

        wpose = self.move_group.get_current_pose().pose

        for pose in poses:
            wpose.pose = pose
            wpose.header.stamp = rospy.Time.now()
            wpose.header.frame_id = frame_id
            waypoints.append(copy.deepcopy(wpose))
        print(waypoints)

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
        display_trajectory_publisher.publish(display_trajectory)


        succeed = self.move_group.execute(plan, wait=True)

        result_msg = PathFollowResult()
        if succeed:
            result_msg.succeed = succeed
            self.action_server.set_succeeded(result_msg)
        else:
            result_msg.succeed = succeed
            self.action_server.set_aborted(result_msg)

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
