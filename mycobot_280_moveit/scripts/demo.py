#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ROS 노드 초기화
rospy.init_node('mycobot_commander')

# 명령을 보낼 퍼블리셔 생성
pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

# 잠시 대기
rospy.sleep(1)

# JointTrajectory 메시지 생성
trajectory = JointTrajectory()
trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

point = JointTrajectoryPoint()
point.positions = [0.1, 0.3, 0.3, 0.4, 0.5, 0.6]  # 각 joint의 목표 위치
point.time_from_start = rospy.Duration(1.0)  # 1초 후에 목표 위치에 도달

trajectory.points.append(point)

# 명령 전송
pub.publish(trajectory)
