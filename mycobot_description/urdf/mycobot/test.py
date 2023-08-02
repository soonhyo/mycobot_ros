import mujoco
import mujoco_viewer

#@title Import packages for plotting and creating graphics
import time
import itertools
import numpy as np
from typing import Callable, NamedTuple, Optional, Union, List

# Graphics and plotting.
import mediapy as media
import matplotlib.pyplot as plt

import cv2
model = mujoco.MjModel.from_xml_path('scene.xml')
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model, 480, 640)

scene_option = mujoco.MjvOption()
scene_option.frame = mujoco.mjtFrame.mjFRAME_GEOM
scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
scene_option.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True

model.opt.gravity = (0, 10, 10)
mujoco.mj_resetDataKeyframe(model, data, 0)

# renderer.enable_depth_rendering()
#renderer.enable_segmentation_rendering()

while True:
    model.geom('floor').rgba[:3] = np.random.rand(3)

    mujoco.mj_forward(model, data)
    renderer.update_scene(data, scene_option=scene_option)

    print('Generalized positions:', data.qpos)

    img = renderer.render()

    # geom_ids = seg[:, :, 0]
    # # Infinity is mapped to -1
    # geom_ids = geom_ids.astype(np.float64) + 1
    # # Scale to [0, 1]
    # geom_ids = geom_ids / geom_ids.max()
    # pixels = 255*geom_ids
    # img = pixels.astype(np.uint8)
    cv2.imshow("im", img)
    cv2.waitKey(1)
