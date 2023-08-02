import mycobot_gym
import gymnasium as gym

env = gym.make("mycobot-v0", render_mode="human")

observation, info = env.reset()

print(observation)
for _ in range(1000):
    action = env.action_space.sample()  # agent policy that uses the observation and info
    observation, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        observation, info = env.reset()

env.close()

print("end")
