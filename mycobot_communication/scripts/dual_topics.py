#!/usr/bin/env python2
import time
import os
import sys
import signal
import threading

import rospy

from pymycobot.mycobot import MyCobot

from mycobot_communication.msg import (
    MydualAngles,
    MydualSetAngles,
    MydualGripperStatus,
    MydualServoStatus,
)

class Watcher:
    """this class solves two problems with multithreaded
    programs in Python, (1) a signal might be delivered
    to any thread (which is just a malfeature) and (2) if
    the thread that gets the signal is waiting, the signal
    is ignored (which is a bug).

    The watcher is a concurrent process (not thread) that
    waits for a signal and the process that contains the
    threads.  See Appendix A of The Little Book of Semaphores.
    http://greenteapress.com/semaphores/

    I have only tested this on Linux.  I would expect it to
    work on the Macintosh and not work on Windows.
    """

    def __init__(self):
        """Creates a child thread, which returns.  The parent
        thread waits for a KeyboardInterrupt and then kills
        the child thread.
        """
        self.child = os.fork()
        if self.child == 0:
            return
        else:
            self.watch()

    def watch(self):
        try:
            os.wait()
        except KeyboardInterrupt:
            # I put the capital B in KeyBoardInterrupt so I can
            # tell when the Watcher gets the SIGINT
            print("KeyBoardInterrupt")
            self.kill()
            sys.exit()

    def kill(self):
        try:
            os.kill(self.child, signal.SIGKILL)
        except OSError:
            pass


class MycobotTopics(object):
    def __init__(self):
        super(MycobotTopics, self).__init__()

        rospy.init_node("Mycobot_topics")
        rospy.loginfo("start ...")
        port_l = rospy.get_param("~port_l", "/dev/ttyACM0")
        port_r = rospy.get_param("~port_r", "/dev/ttyUSB0")
        ports = [port_l, port_r]
        # port = rospy.get_param("~port", os.popen("ls /dev/ttyACM*").readline()[:-1])
        baud = rospy.get_param("~baud", 115200)
        rospy.loginfo("%s,%s" % (ports, baud))
        self.mcs = [MyCobot(port, baud) for port in ports]
        self.lock = [threading.Lock() for _ in ports]
        self.angles = [None for _ in range(12)]
        self.rate = rospy.Rate(20)

    def start(self):
        ga1 = threading.Thread(target=self.get_real_angles, args=(0,))
        ga2 = threading.Thread(target=self.get_real_angles, args=(1,))
        pa = threading.Thread(target=self.pub_real_angles)

        sa = threading.Thread(target=self.sub_set_angles)

        sg = threading.Thread(target=self.sub_gripper_status)
        ss = threading.Thread(target=self.sub_servo_status)

        # sp = threading.Thread(target=self.sub_pump_status)

        pa.setDaemon(True)
        pa.start()
        sa.setDaemon(True)
        sa.start()
        sg.setDaemon(True)
        sg.start()
        ss.setDaemon(True)
        ss.start()

        ga1.setDaemon(True)
        ga2.start()
        ga2.setDaemon(True)
        ga2.start()

        pa.join()
        sa.join()
        sg.join()
        ss.join()

        ga1.join()
        ga2.join()

    def get_real_angles(self, idx):
        while not rospy.is_shutdown():
            self.lock[idx].acquire()
            angles = self.mcs[idx].get_angles()
            self.lock[idx].release()
            if angles:
                for i, angle in enumerate(angles):
                    self.angles[idx*6+i] = angle

    def pub_real_angles(self):
        """Publish real angle"""
        """发布真实角度"""
        pub = rospy.Publisher("Mycobot/angles_real",
                              MycobotAngles, queue_size=15)
        ma = MycobotAngles()
        while not rospy.is_shutdown():
            if not (None in self.angles):
                ma.joint_0 = self.angles[0]
                ma.joint_1 = self.angles[1]
                ma.joint_2 = self.angles[2]
                ma.joint_3 = self.angles[3]
                ma.joint_4 = self.angles[4]
                ma.joint_5 = self.angles[5]

                ma.joint_6 = self.angles[6]
                ma.joint_7 = self.angles[7]
                ma.joint_8 = self.angles[8]
                ma.joint_9 = self.angles[9]
                ma.joint_10 = self.angles[10]
                ma.joint_11 = self.angles[11]
                pub.publish(ma)
                # time.sleep(0.25)
                self.rate.sleep()

    def set_angles(self,idx, sp, angles):
        self.mcs[idx].send_angles(angles, sp)

    def sub_set_angles(self):
        """subscription angles"""
        """订阅角度"""
        def callback(data):
            angles = [
                data.joint_0,
                data.joint_1,
                data.joint_2,
                data.joint_3,
                data.joint_4,
                data.joint_5,
                data.joint_6,
                data.joint_7,
                data.joint_8,
                data.joint_9,
                data.joint_10,
                data.joint_11,

            ]
            sp = int(data.speed)
            sa1 = threading.Thread(target=self.set_angles, args=(0, sp, angles[:6]))
            sa2 = threading.Thread(target=self.set_angles, args=(1, sp, angles[6:]))
            sa1.setDaemon(True)
            sa2.setDaemon(True)
            sa1.start()
            sa2.start()
            sa1.join()
            sa2.join()

        sub = rospy.Subscriber(
            "Mycobot/angles_goal", MycobotSetAngles, callback=callback
        )
        rospy.spin()

    def set_gripper(self, idx, status):
        self.mcs[idx].set_gripper_state(status, 80)

    def sub_gripper_status(self):
        """Subscribe to Gripper Status"""
        """订阅夹爪状态"""
        def callback(data):
            gs1 = threading.Thread(target=self.set_gripper, args=(0, data.lgripper, ))# 0:off, 1:on for left
            gs2 = threading.Thread(target=self.set_gripper, args=(1, data.rgripper, ))# 0:off, 1:on for right
            gs1.setDaemon(True)
            gs2.setDaemon(True)
            gs1.start()
            gs2.start()
            gs1.join()
            gs2.join()

        sub = rospy.Subscriber(
            "Mycobot/gripper_status", MycobotGripperStatus, callback=callback
        )
        rospy.spin()

    def sub_servo_status(self):
        def callback(data):
            if data.Status:
                self.mcs[0].release_all_servos()
                self.mcs[1].release_all_servos()
                rospy.loginfo("servo off")
            else:
                self.mcs[0].send_angles(self.angles[:6], 0)
                self.mcs[1].send_angles(self.angles[6:], 0)
                rospy.loginfo("servo on")

        sub = rospy.Subscriber(
            "Mycobot/servo_status", MycobotServoStatus, callback=callback
        )
        rospy.spin()


if __name__ == "__main__":
    Watcher()
    mb_topics = MycobotTopics()
    mb_topics.start()
    # while True:
    #     mb_topics.pub_real_coords()
    # mb_topics.sub_set_angles()
    pass
