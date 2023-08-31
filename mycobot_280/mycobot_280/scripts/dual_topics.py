#!/usr/bin/env python2
import time
import os
import sys
import signal
import threading

import rospy

from pymycobot.mycobot import MyCobot

from mycobot_communication.msg import (
    MycobotAngles,
    MycobotCoords,
    MycobotSetAngles,
    MycobotSetCoords,
    MycobotGripperStatus,
    MycobotPumpStatus,
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

    def start(self):
        pa = threading.Thread(target=self.pub_real_angles)
        sa = threading.Thread(target=self.sub_set_angles)

        sg = threading.Thread(target=self.sub_gripper_status)

        pa = threading.Thread(target=self.pub_real_angles)
        sa = threading.Thread(target=self.sub_set_angles)

        sg = threading.Thread(target=self.sub_gripper_status)

        # sp = threading.Thread(target=self.sub_pump_status)

        pa.setDaemon(True)
        pa.start()
        pb.setDaemon(True)
        pb.start()
        sa.setDaemon(True)
        sa.start()
        sb.setDaemon(True)
        sb.start()
        sg.setDaemon(True)
        sg.start()
        sp.setDaemon(True)
        sp.start()

        pa.join()
        pb.join()
        sa.join()
        sb.join()
        sg.join()
        sp.join()

    def pub_real_angles(self):
        """Publish real angle"""
        """发布真实角度"""
        pub = rospy.Publisher("Mycobot/angles_real",
                              MycobotAngles, queue_size=15)
        ma = MycobotAngles()
        while not rospy.is_shutdown():
            self.lock.acquire()
            angles = self.mc.get_angles()
            self.lock.release()
            if angles:
                ma.joint_0 = angles[0]
                ma.joint_1 = angles[1]
                ma.joint_2 = angles[2]
                ma.joint_3 = angles[3]
                ma.joint_4 = angles[4]
                ma.joint_5 = angles[5]

                ma.joint_6 = angles[6]
                ma.joint_7 = angles[7]
                ma.joint_8 = angles[8]
                ma.joint_9 = angles[9]
                ma.joint_10 = angles[10]
                ma.joint_11 = angles[11]
                ma.joint_12 = angles[12]
                pub.publish(ma)
            time.sleep(0.25)

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
                data.joint_12,
                
            ]
            sp = int(data.speed)
            self.mc.send_angles(angles, sp)

        sub = rospy.Subscriber(
            "Mycobot/angles_goal", MycobotSetAngles, callback=callback
        )
        rospy.spin()

    def sub_gripper_status(self):
        """Subscribe to Gripper Status"""
        """订阅夹爪状态"""
        def callback(data):
            if data.Status:
                self.mc.set_gripper_state(0, 80)
            else:
                self.mc.set_gripper_state(1, 80)

        sub = rospy.Subscriber(
            "Mycobot/gripper_status", MycobotGripperStatus, callback=callback
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
