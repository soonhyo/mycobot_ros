#!/usr/bin/env python3
import time
import rospy
from mycobot_communication.srv import *
import threading
from pymycobot.mycobot import MyCobot

mc = None


def create_handle():
    global mc
    global lock
    lock = threading.Lock()
    rospy.init_node("mycobot_services")
    rospy.loginfo("start ...")
    port = rospy.get_param("~port")
    baud = rospy.get_param("~baud")
    rospy.loginfo("%s,%s" % (port, baud))
    mc = MyCobot(port, baud)


def create_services():
    rospy.Service("set_joint_angles", SetAngles, set_angles)
    rospy.Service("get_joint_angles", GetAngles, get_angles)
    rospy.Service("set_joint_coords", SetCoords, set_coords)
    rospy.Service("get_joint_coords", GetCoords, get_coords)
    rospy.Service("switch_gripper_status", GripperStatus, switch_status)
    rospy.Service("switch_pump_status", PumpStatus, toggle_pump)
    rospy.loginfo("ready")
    rospy.spin()


def set_angles(req):
    angles = [
        req.joint_1,
        req.joint_2,
        req.joint_3,
        req.joint_4,
        req.joint_5,
        req.joint_6,
    ]
    sp = req.speed

    if mc:
        lock.acquire()
        mc.send_angles(angles, sp)
        lock.release()
    return SetAnglesResponse(True)


def get_angles(req):
    if mc:
        lock.acquire()
        angles = mc.get_angles()
        lock.release()

        return GetAnglesResponse(*angles)


def set_coords(req):
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

    if mc:
        lock.acquire()
        mc.send_coords(coords, sp, mod)
        lock.release()
    return SetCoordsResponse(True)


def get_coords(req):
    if mc:
        lock.acquire()
        coords = mc.get_coords()
        lock.release()
        return GetCoordsResponse(*coords)


def switch_status(req):
    if mc:
        lock.acquire()
        if req.Status:
            mc.set_gripper_state(0, 80)
        else:
            mc.set_gripper_state(1, 80)
        lock.release()
    return GripperStatusResponse(True)


def toggle_pump(req):
    if mc:
        lock.acquire()
        if req.Status:
            mc.set_basic_output(2, 0)
            mc.set_basic_output(5, 0)
        else:
            mc.set_basic_output(2, 1)
            mc.set_basic_output(5, 1)
        lock.release()
    return PumpStatusResponse(True)


robot_msg = """
MyCobot Status
--------------------------------
Joint Limit:
    joint 1: -170 ~ +170
    joint 2: -170 ~ +170
    joint 3: -170 ~ +170
    joint 4: -170 ~ +170
    joint 5: -170 ~ +170
    joint 6: -180 ~ +180

Connect Status: %s

Servo Infomation: %s

Servo Temperature: %s

Atom Version: %s
"""


def output_robot_message():
    connect_status = False
    servo_infomation = "unknown"
    servo_temperature = "unknown"
    atom_version = "unknown"

    if mc:
        lock.acquire()
        cn = mc.is_controller_connected()
        if cn == 1:
            connect_status = True
        time.sleep(0.1)
        si = mc.is_all_servo_enable()
        if si == 1:
            servo_infomation = "all connected"
        lock.release()
    print(
        robot_msg % (connect_status, servo_infomation, servo_temperature, atom_version)
    )


if __name__ == "__main__":
    # print(MyCobot.__dict__)
    create_handle()
    output_robot_message()
    create_services()
