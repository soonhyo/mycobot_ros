import os
import torch as th
import torch.nn as nn
from gymnasium import spaces

from stable_baselines3 import PPO
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.vec_env import DummyVecEnv

import mycobot_gym
import gymnasium as gym
import argparse


class CustomCNN(BaseFeaturesExtractor):
    """
    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """

    def __init__(self, observation_space: spaces.Dict, features_dim: int = 256):
        super().__init__(observation_space, features_dim)
        # We assume CxHxW images (channels first)
        # Re-ordering will be done by pre-preprocessing or wrapper
        n_input_channels = observation_space.spaces['image'].shape[0]  # use 'image' space shape
        self.cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 32, kernel_size=8, stride=4, padding=0),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Flatten(),
        )

        # Compute shape by doing one forward pass
        with th.no_grad():
            n_flatten = self.cnn(
                th.as_tensor(observation_space.spaces['image'].sample()[None]).float()  # use 'image' space sample
            ).shape[1]

        self.linear = nn.Sequential(nn.Linear(n_flatten, features_dim), nn.ReLU())

    def forward(self, observations: th.Tensor) -> th.Tensor:
        return self.linear(self.cnn(observations['image']))  # use 'image' observations


class CustomPolicy(ActorCriticPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomPolicy, self).__init__(*args, **kwargs,
                                           features_extractor_class=CustomCNN,
                                           features_extractor_kwargs=dict(features_dim=64*64*3),
                                           net_arch=[64*64*3+18, 256, 128])


parser = argparse.ArgumentParser()
parser.add_argument("--what", type=str, default="train")
args = parser.parse_args()

env = gym.make("mycobot-v0", render_mode="human")
env = DummyVecEnv([lambda: env])

root_dir = os.path.dirname(os.path.abspath(__file__))
model = PPO(CustomPolicy, env, n_steps=2048, batch_size=2, gamma=0.99, verbose=1, tensorboard_log=root_dir+"/saved")

if args.what == "train":
    print("start training...")
    model.learn(total_timesteps=1000)
    model.save("ppo_mycobot")

model = PPO.load("ppo_mycobot")

print("start testing...")
obs = env.reset()
for _ in range(1000):
    action, _states = model.predict(obs)
    obs, rewards, done, info = env.step(action)
    if done:
        obs = env.reset()

env.close()

print("end")
