import time

import os
import sys
import numpy as np

import rospy
from rospkg import RosPack
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import JointState, Image, CameraInfo

import mujoco
import mujoco.viewer

import itertools
import cv2

m = mujoco.MjModel.from_xml_path('scene.xml')
#
# xml = """
# <mujoco>
#   <worldbody>
#     <light name="top" pos="0 0 1"/>
#     <body name="box_and_sphere" euler="0 0 -30">
#       <joint name="swing" type="hinge" axis="1 -1 0" pos="-.2 -.2 -.2"/>
#       <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
#       <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
#     </body>
#   </worldbody>
# </mujoco>
# """
# m = mujoco.MjModel.from_xml_string(xml)
d = mujoco.MjData(m)
r = mujoco.Renderer(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        mujoco.mj_step(m, d)

        # Example modification of a viewer option: toggle contact points every two seconds.
        # with viewer.lock():
        #     viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.

        mujoco.mj_forward(m, d)
        r.update_scene(d, camera="camera")

        viewer.sync()

        pixels = r.render()
        cv2.imshow("img", pixels)
        cv2.waitKey(1)



        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
