import os

import numpy as np

from gymnasium import utils
from gymnasium.envs.mujoco import MujocoEnv
from gymnasium.spaces import Box

import cv2

DEFAULT_CAMERA_CONFIG = {
    "trackbodyid": -1,
    "distance": 4.0,
}


class MyCobotEnv(MujocoEnv, utils.EzPickle):

    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ],
        "render_fps": 100,
    }

    def __init__(self, **kwargs):
        utils.EzPickle.__init__(self, **kwargs)
        observation_space = Box(low=-np.inf, high=np.inf, shape=(18,), dtype=np.float64)
        MujocoEnv.__init__(
            self,
            os.path.dirname(os.path.abspath(__file__)) + "/assets/scene.xml",
            5,
            observation_space=observation_space,
            camera_id=0,
            default_camera_config=DEFAULT_CAMERA_CONFIG,
            **kwargs,
        )

    def step(self, a):
        vec = self.get_body_com("box1") - self.get_body_com("link7")

        reward_dist = -np.linalg.norm(vec)
        reward_ctrl = -np.square(a).sum()
        reward = reward_dist + 0.1 * reward_ctrl

        self.do_simulation(a, self.frame_skip)
        if self.render_mode == "human":
            self.render()
        else:
            img = self.render()
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv2.imshow("img", img)
            cv2.waitKey(1)
        ob = self._get_obs()
        return (
            ob,
            reward,
            False,
            False,
            dict(reward_dist=reward_dist, reward_ctrl=reward_ctrl),
        )

    def reset_model(self):
        qpos = self.init_qpos
        qvel = self.init_qvel + self.np_random.uniform(
            low=-0.005, high=0.005, size=self.model.nv
        )
        qvel[-6:] = 0
        self.set_state(qpos, qvel)
        return self._get_obs()

    def _get_obs(self):
        return np.concatenate(
            [
                self.data.qpos.flat[:6],
                self.data.qvel.flat[:6],
                self.get_body_com("link7"),
                self.get_body_com("box1"),
            ]
        )
