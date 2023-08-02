import os
import mycobot_gym
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
import argparse


parser = argparse.ArgumentParser()
parser.add_argument("--what", type=str, default="train")
args = parser.parse_args()
# 환경 생성
env = gym.make("mycobot-v0", render_mode="human")

# VecEnv 인스턴스를 생성
# Stable Baselines3는 벡터화된 환경을 요구함
env = DummyVecEnv([lambda: env])

# 모델 생성
root_dir = os.path.dirname(os.path.abspath(__file__))
model = PPO("MlpPolicy", env, n_steps=2048, batch_size=64, gamma=0.99, verbose=1, tensorboard_log=root_dir+"/saved")

if args.what == "train":
    print("start training...")
    # 모델 학습
    model.learn(total_timesteps=100000)
    # 학습된 모델 저장
    model.save("ppo_mycobot")

# 저장된 모델 불러오기
model = PPO.load("ppo_mycobot")

print("start testing...")
# 결과 테스트
obs = env.reset()
for _ in range(1000):
    action, _states = model.predict(obs)
    obs, rewards, done, info = env.step(action)
    if done:
        obs = env.reset()

env.close()

print("end")
