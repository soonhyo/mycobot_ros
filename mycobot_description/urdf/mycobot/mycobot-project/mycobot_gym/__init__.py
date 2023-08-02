from gymnasium.envs.registration import register

register(
    id="mycobot-v0",
    entry_point="mycobot_gym.envs:MyCobotEnv",
)
