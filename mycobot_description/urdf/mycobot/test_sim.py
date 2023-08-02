import mujoco
import mujoco_py
import cv2

model = mujoco_py.load_model_from_path("scene.xml")
sim = mujoco_py.MjSim(model)

 ## a is a tuple if depth is True and a numpy array if depth is False ##
a = sim.render(width=200, height=200, camera_name='camera', depth=True)
rgb_img = a[0]
depth_img = a[1]


cv2.imshow("img", rgb_img)
cv2.waitKey(0)
