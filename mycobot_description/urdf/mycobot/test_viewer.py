import mujoco
import mujoco_viewer
import cv2
model = mujoco.MjModel.from_xml_path('scene.xml')
data = mujoco.MjData(model)



viewer = mujoco_viewer.MujocoViewer(model, data, 'offscreen')

for _ in range(10):
    mujoco.mj_forward(model, data)
    img = viewer.read_pixels(camid=0)

    cv2.imshow("img", img)
    cv2.waitKey(0)
## do something cool with img
