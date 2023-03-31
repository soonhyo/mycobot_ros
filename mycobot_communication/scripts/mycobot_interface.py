#!/usr/bin/env python
import time
import os
import sys
import signal
import serial
import threading
import math
if sys.version_info < (3,0):
    from itertools import izip_longest # TODO: python3 is zip_longest
    zip_longest = izip_longest
else:
    from itertools import zip_longest

from pymycobot.mycobot import MyCobot

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback, GripperCommandAction, GripperCommandResult, GripperCommandFeedback
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import SetBool, SetBoolResponse, Empty
import tf
import numpy as np

from mycobot_communication.srv import *

class MycobotInterface(object):

    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = rospy.get_param("~baud", 115200)
        debug_onoff = rospy.get_param("~mycobot_debug",True)
        self.model = rospy.get_param("~model", '280')
        self.vel_rate = rospy.get_param("~vel_rate", 32.0) # bit/rad
        self.min_vel = rospy.get_param("~min_vel", 10) # bit, for the bad velocity tracking of mycobot.
        rospy.loginfo("Connect mycobot on %s,%s" % (port, baud))
        self.mc = MyCobot(port, baud, debug=debug_onoff)
        self.lock = threading.Lock()

        self.set_srv =[]

        self.joint_angle_pub = rospy.Publisher("joint_states", JointState, queue_size=5)
        self.real_angles = None
        self.joint_command_sub = rospy.Subscriber("joint_command", JointState, self.joint_command_cb)

        self.pub_end_coord = rospy.get_param("~pub_end_coord", False)
        if self.pub_end_coord:
            self.end_coord_pub = rospy.Publisher("end_coord", PoseStamped, queue_size=5)

        self.servo_srv = rospy.Service("set_servo", SetBool, self.set_servo_cb)

        self.open_gripper_srv = rospy.Service("open_gripper", Empty, self.open_gripper_cb)
        self.close_gripper_srv = rospy.Service("close_gripper", Empty, self.close_gripper_cb)
        self.gripper_is_moving = False
        self.gripper_value = None
        self.get_gripper_state = False

        # action server for joint and gripper
        self.joint_as = actionlib.SimpleActionServer("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction, execute_cb=self.joint_as_cb)
        self.joint_as.start()

        self.get_joint_state = True

        self.gripper_as = actionlib.SimpleActionServer("gripper_controller/gripper_command", GripperCommandAction, execute_cb=self.gripper_as_cb)
        self.gripper_as.start()

        # service server
        self.create_services()
    def run(self):

        r = rospy.Rate(rospy.get_param("~joint_state_rate", 20.0)) # hz
        while not rospy.is_shutdown():
            try:        
                # get real joint from MyCobot
                self.lock.acquire()
                if self.mc.is_moving():
                    self.lock.release()
                    continue
                self.lock.release()
                if self.get_joint_state:
                    # real_angles = self.mc.get_angles()
                    # the duration is over the sending loop, not good
                    self.lock.acquire()
                    real_angles = self.mc.get_angles()
                    self.lock.release()
                    
                    if len(real_angles) != 6: # we assume mycobot only have 6 DoF of joints
                        rospy.logwarn("empty joint angles!!!")
                        rospy.loginfo("real_angles error "+ str(real_angles))
                    else:
                        self.real_angles = real_angles

                if self.real_angles:
                    rospy.logdebug_throttle(1.0, "get real angles from mycobot")

                    msg = JointState()
                    msg.header.stamp = rospy.get_rostime()

                    for i, ang in enumerate(self.real_angles):
                        if (i+1) == len(self.real_angles): #last robot joint
                            msg.name.append('joint' + str(i+1)+'output_to_'+'joint'+str(i+1))
                        else:
                            msg.name.append('joint' + str(i+2)+'_to_'+'joint'+str(i+1))
                        # if (i+1) == (len(self.real_angles) -1): #last robot joint
                        #     msg.name.append('joint' + str(i+1)+'output_to_'+'joint'+str(i+1))
                        # else if (i+1) < (len(self.real_angles) -1):
                        #     msg.name.append('joint' + str(i+2)+'_to_'+'joint'+str(i+1))
                        # else:
                        #     msg.name.append('end_effector')

                        msg.position.append(ang / 180.0 * math.pi)
                    self.joint_angle_pub.publish(msg)

                # get gripper state
                # Note: we only retreive the gripper state when doing the grasp action.
                # We find following polling function will cause the failure of get_angles() for Mycobot Pro 320.
                # This makes the publish of joint state decrease from 20Hz to 2Hz for MyCobot Pro 320.
                if self.get_gripper_state:
                    self.gripper_is_moving = self.mc.is_gripper_moving()
                    self.gripper_value = self.mc.get_gripper_value()


                # get end-effector if necessary (use tf by ros in default)
                if self.pub_end_coord:
                    coords = self.mc.get_coords()
                    if coords:
                        msg = PoseStamped
                        msg.header.stamp = rospy.get_rostime()
                        msg.pose.position.x = coords[0]
                        msg.pose.position.y = coords[1]
                        msg.pose.position.z = coords[2]
                        q = tf.transformations.quaternion_from_euler(coords[3], coords[4], coords[5])
                        msg.poseq.quaternion.x = q[0]
                        msg.poseq.quaternion.y = q[1]
                        msg.poseq.quaternion.z = q[2]
                        msg.poseq.quaternion.w = q[3]
                        self.end_coord_pub.publish(msg)

                r.sleep()
            except serial.serialutil.SerialException as e:
                rospy.logerr(e)
                if self.mc.is_power_on() == 0:
                    rospy.loginfo("Atom is down, repower on the atom")
                    self.mc.power_on()
                    time.sleep(0.1)
                    if self.mc.is_power_on() == 1:
                        rospy.loginfo("Atom is powered on")
                    else:
                        rospy.loginfo("Atom is not powered on")
                        break
                continue

    def create_services(self):
        rospy.Service("set_joint_angles", SetAngles, self.set_angles)
        rospy.Service("get_joint_angles", GetAngles, self.get_angles)
        rospy.Service("set_joint_coords", SetCoords, self.set_coords)
        rospy.Service("get_joint_coords", GetCoords, self.get_coords)
        rospy.Service("switch_gripper_status", GripperStatus, self.switch_status)
        rospy.Service("switch_pump_status", PumpStatus, self.toggle_pump)
        rospy.loginfo("services server ready")
        
    def set_angles(self,req):
        angles = [
            req.joint_1,
            req.joint_2,
            req.joint_3,
            req.joint_4,
            req.joint_5,
            req.joint_6,
        ]
        sp = req.speed

        if self.mc:
            self.lock.acquire()
            self.mc.send_angles(angles, sp)
            self.lock.release()


        return SetAnglesResponse(True)


    def get_angles(self, req):
        if self.mc:
            self.lock.acquire()
            angles = self.mc.get_angles()
            self.lock.release()
            return GetAnglesResponse(*angles)


    def set_coords(self, req):
        coords = [
            req.x,
            req.y,
            req.z,
            req.rx,
            req.ry,
            req.rz,
        ]
        sp = req.speed
        mod = req.model
        # data = [coords, sp, mod]
        if self.mc:
            self.lock.acquire()
            rospy.sleep(0.5)
            self.mc.send_coords(coords, sp, mod)
            self.lock.release()
        # self.set_srv = data
        return SetCoordsResponse(True)


    def get_coords(self, req):
        if self.mc:
            self.lock.acquire()
            coords = self.mc.get_coords()
            self.lock.release()
            return GetCoordsResponse(*coords)

    def switch_status(self, req):
        if self.mc:
            self.lock.acquire()
            if req.Status:
                self.mc.set_gripper_state(0, 80)
            else:
                self.mc.set_gripper_state(1, 80)
            self.lock.release()
        return GripperStatusResponse(True)

    def toggle_pump(self, req):
        if self.mc:
            self.lock.acquire()
            if req.Status:
                self.mc.set_basic_output(2, 0)
                self.mc.set_basic_output(5, 0)
            else:
                self.mc.set_basic_output(2, 1)
                self.mc.set_basic_output(5, 1)
            self.lock.release()
        return PumpStatusResponse(True)

    def joint_command_cb(self, msg):
        angles = self.real_angles
        vel = 50 # deg/s, hard-coding
        for n, p, v in zip_longest(msg.name, msg.position, msg.velocity):
            id = int(n[-1]) - 1
            if 'joint' in n and id >= 0 and id < len(angles):
                if math.fabs(p) < 190.0 / 180 * math.pi: # 190 should be  retrieved from API
                    angles[id] = p * 180 / math.pi
                else:
                    rospy.logwarn("%s exceeds the limit, %f", n, p)
            if v:
                v = v * 180 / math.pi
                if v < vel:
                    vel = v

        print(angles, vel)
        self.lock.acquire()
        self.mc.send_angles(angles, vel)
        self.lock.release()

    def set_servo_cb(self, req):
        if req.data:
            self.lock.acquire()
            self.mc.send_angles(self.real_angles, 0)
            self.lock.release()
            rospy.loginfo("servo on")
        else:
            self.lock.acquire()
            self.mc.release_all_servos()
            self.lock.release()
            rospy.loginfo("servo off")

        return SetBoolResponse(True, "")

    def open_gripper_cb(self, req):
        self.lock.acquire()
        self.mc.set_gripper_state(0, 80)
        self.lock.release()
        rospy.loginfo("open gripper")

    def close_gripper_cb(self, req):
        self.lock.acquire()
        self.mc.set_gripper_state(1, 80)
        self.lock.release()
        rospy.loginfo("close gripper")

    def joint_as_cb(self, goal):

        if '320' in self.model: # workaround for mycobot pro 320
            self.get_joint_state = False

        # Error case1
        if not self.real_angles:
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            msg = "Real joint angles are empty!"
            rospy.logerr(msg);
            self.joint_as.set_aborted(res, msg)
            return

        # Error case2
        if len(self.real_angles) != len(goal.trajectory.joint_names):
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            msg = "Incoming trajectory joints do not match the joints of the controller"
            rospy.logerr(msg);
            self.joint_as.set_aborted(res, msg)
            return

        # Error case3: make sure trajectory is not empty
        if len(goal.trajectory.points) == 0:
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            msg = "Incoming trajectory is empty"
            rospy.logerr(msg);
            self.joint_as.set_aborted(res, msg)
            return


        # correlate the joints we're commanding to the joints in the message
        durations = []

        # num_points = len(goal.trajectory.points);
        points = goal.trajectory.points
        # find out the duration of each segment in the trajectory
        durations.append(points[0].time_from_start)

        for i in range(1, len(goal.trajectory.points)):
            durations.append(points[i].time_from_start - points[i-1].time_from_start);

        # Error case4: empty
        if not points[0].positions:
             msg = "First point of trajectory has no positions"
             res = FollowJointTrajectoryResult()
             res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
             rospy.logerr(msg);
             self.joint_as.set_aborted(res, msg)
             return

        trajectory = []
        time = rospy.Time.now() + rospy.Duration(0.01)

        for i in range(len(points)):
            seg_start_time = 0;
            seg_duration = 0;
            seg = {}

            if goal.trajectory.header.stamp == rospy.Time():
                seg['start_time'] = (time + points[i].time_from_start) - durations[i]
            else:
                seg['start_time'] = (goal.trajectory.header.stamp + points[i].time_from_start) - durations[i]

            seg['end_time'] = seg['start_time'] + durations[i];

            # Checks that the incoming segment has the right number of elements.
            if len(points[i].velocities) > 0 and len(points[i].velocities) != len(goal.trajectory.joint_names):
                msg = "Command point " + str(i+1) + " has wrong amount of velocities"
                res = FollowJointTrajectoryResult()
                res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
                rospy.logerr(msg);
                self.joint_as.set_aborted(res, msg)
                return

            if len(points[i].positions) != len(goal.trajectory.joint_names):
                msg = "Command point " + str(i+1) + " has wrong amount of positions"
                res = FollowJointTrajectoryResult()
                res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
                rospy.logerr(msg);
                self.joint_as.set_aborted(res, msg)
                return

            if len(points[i].velocities) > 0:
                seg['velocities'] = points[i].velocities
            seg['positions'] = points[i].positions

            trajectory.append(seg);


        ## wait for start
        rospy.loginfo("Trajectory start requested at %.3lf, waiting...", goal.trajectory.header.stamp.to_sec())
        r = rospy.Rate(100)
        while (goal.trajectory.header.stamp - time).to_sec() > 0:
            time = rospy.Time.now()
            r.sleep()
        total_duration = sum(map(lambda du: du.to_sec(), durations))
        rospy.loginfo("Trajectory start time is %.3lf, end time is %.3lf, total duration is %.3lf", time.to_sec(),  time.to_sec() + total_duration, total_duration);


        feedback = FollowJointTrajectoryFeedback()
        feedback.joint_names = goal.trajectory.joint_names;
        feedback.header.stamp = time;

        for i, seg in enumerate(trajectory):

            if durations[i] == 0:
                rospy.logdebug("skipping segment %d with duration of 0 seconds", i);
                continue;

            target_angles =  np.array(seg['positions']) * 180 / np.pi
            actual_angles = np.array(self.real_angles);

            # workaround to solve bad joint velocity control
            if len(trajectory)  == 1: # only has the goal angle position, calculate the average velocity
                vel = (target_angles - np.array(self.real_angles)) / durations[i].to_sec() * np.pi / 180.0
                vel = int(np.max(np.abs(vel)) * self.vel_rate )
            else:
                vel = int(np.max(np.abs(seg['velocities'])) * self.vel_rate)
                if vel < self.min_vel: # workaround to solve the bad velocity tracking of mycobot
                    vel = self.min_vel
                # vel = 0 # zero in pymycobot is the max speed

            self.lock.acquire()
            self.mc.send_angles(target_angles.tolist(), vel)
            self.lock.release()

            # workaround: pure feedforwad style to address the polling/sending conflict problem in mycobot pro 320
            if not self.get_joint_state:
                if len(trajectory)  == 1: # only has the goal angle position
                    self.get_joint_state= True
                else:
                    self.real_angles = target_angles.tolist()

            while time.to_sec() < seg["end_time"].to_sec():

                rospy.logdebug("Current segment is %d time left %f cur time %f", i, (durations[i] - (time - seg["start_time"])).to_sec(), time.to_sec());

                # check if new trajectory was received, if so abort current trajectory execution
                # by setting the goal to the current position
                if self.joint_as.is_preempt_requested():

                    self.lock.acquire()
                    self.mc.send_angles(self.real_angles, 0)
                    self.lock.release()

                    self.joint_as.set_preempted()
                    if self.joint_as.is_new_goal_available():
                        rospy.logwarn("New trajectory received. Aborting old trajectory.");
                    else:
                        rospy.logwarn("Canceled trajectory following action");

                    self.get_joint_state = True
                    return;

                if (time - feedback.header.stamp).to_sec() > 0.1: # 10 Hz

                    feedback.header.stamp = time;
                    feedback.desired.positions = (target_angles / 180 * np.pi).tolist()
                    feedback.actual.positions = (actual_angles / 180 * np.pi).tolist()
                    feedback.error.positions = ((target_angles - actual_angles) / 180 * np.pi).tolist()
                    self.joint_as.publish_feedback(feedback);


                r.sleep();
                time = rospy.Time.now()

            # Verify trajectory constraints
            for tol in goal.path_tolerance:
                index = goal.trajectory.joint_names.index(tol.name)
                pos_err = np.fabs(target_angles - actual_angles)[index]  / 180 * np.pi;

                if tol.position > 0 and pos_err > tol.position:
                    msg = "Unsatisfied position tolerance for " + tol.name + \
                          ", trajectory point"  + str(i+1) + ", " + str(pos_err) + \
                          " is larger than " + str(tol.position);


                    rospy.logwarn(msg);
                    res = FollowJointTrajectoryResult()
                    res.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                    rospy.logwarn(msg);
                    self.joint_as.set_aborted(res, msg)

                    self.get_joint_state = True
                    return;

        # Checks that we have ended inside the goal constraints
        if not self.get_joint_state:
            actual_angles = np.array(self.mc.get_angles())
        for tol in goal.goal_tolerance:
            index = goal.trajectory.joint_names.index(tol.name)

            pos_err = np.fabs(target_angles - actual_angles)[index] / 180 * np.pi
            if tol.position > 0 and pos_err > tol.position:
                msg = "Aborting because " + tol.name + \
                      " wound up outside the goal constraints, " + \
                      str(pos_err) + " is larger than " + str(tol.position)

                rospy.logwarn(msg);
                res = FollowJointTrajectoryResult()
                res.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                rospy.logwarn(msg);
                self.joint_as.set_aborted(res, msg)
                return;


        msg = "Trajectory execution successfully completed"
        rospy.loginfo(msg)
        res = FollowJointTrajectoryResult()
        res.error_code = FollowJointTrajectoryResult.SUCCESSFUL;
        self.joint_as.set_succeeded(res, msg);

        self.get_joint_state = True

    def gripper_as_cb(self, goal):

        goal_state = (int)(goal.command.position)
        if not (goal_state == 0 or goal_state == 1):
            res = GripperCommandResult()
            res.position = self.gripper_value
            res.stalled = True
            res.reached_goal = False
            msg = "We only support 1 (totally close) or 0 (totally open) for gripper action"
            rospy.logerr(msg);
            self.gripper_as.set_aborted(res, msg)
            return

        feedback = GripperCommandFeedback()

        self.get_gripper_state = True # start retrive the grasping data
        if '320' in self.model: # workaround for mycobot pro 320
            self.get_joint_state = False # stop polling joint angles
            time.sleep(0.1) # wait for the finish of last joint angles polling

        self.lock.acquire()
        self.mc.set_gripper_state(goal_state, 100) # first arg is the flag 0 - open, 1 - close; second arg is the speed
        self.lock.release()

        self.get_joint_state = True # resume polling joint state if necessary

        t = rospy.Time(0)
        rospy.sleep(0.3) # wait for the gripper to start moving

        r = rospy.Rate(20) # 20 Hz
        while not rospy.is_shutdown():

            rospy.logdebug("Current gripper value is  %d state is %d", self.gripper_value, self.gripper_is_moving);

            if self.gripper_as.is_preempt_requested():

                self.gripper_as.set_preempted()
                self.get_gripper_state = False
                return;

            if (rospy.Time.now() - t).to_sec() > 0.1: # 10 Hz
                feedback.position = self.gripper_value
                feedback.stalled = False
                feedback.stalled = False
                self.gripper_as.publish_feedback(feedback);
                t = rospy.Time.now()

            if self.gripper_is_moving == 0: # not moving
                self.get_gripper_state = False
                msg = "Gripper stops moving"
                rospy.loginfo(msg)
                res = GripperCommandResult()
                res.position = self.gripper_value
                res.stalled = True
                res.reached_goal = True
                self.gripper_as.set_succeeded(res, msg);
                break


            r.sleep();


if __name__ == "__main__":
    rospy.init_node("mycobot_topics")
    mc_inteface = MycobotInterface()
    mc_inteface.run()
    pass
