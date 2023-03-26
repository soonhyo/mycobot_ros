#!/bin/env/python3
from moveit_ros_planning_interface._moveit_robot_interface import RobotInterface
ri = RobotInterface("/robot_description")
g = ri.get_group_names()
print(g)
print(ri.get_group_joint_names(g[1]))
print(ri.get_group_joint_tips(g[0]))