#!/usr/bin/env python3

import numpy as np
from scipy.interpolate import CubicSpline

def generate_trajectory(joint_positions, time_steps, joint_velocity_limits):
    num_joints = len(joint_positions[0])
    trajectories = []

    for i in range(num_joints):
        joint_positions_i = [pos[i] for pos in joint_positions]
        cs = CubicSpline(time_steps, joint_positions_i)

        # calculate the derivative (velocity) of the spline and ensure it doesn't exceed the limit
        velocity_i = cs(time_steps, 1)
        if np.any(np.abs(velocity_i) > joint_velocity_limits[i]):
            print(f"Joint {i} velocity limit exceeded!")
            return None

        trajectories.append(cs)

    return trajectories

# usage
joint_positions = [
    [0, 0, 0, 0, 0, 0],  # initial joint positions
    [1, 2, 3, 4, 5, 6],  # intermediate joint positions
    [0, 0, 0, 0, 0, 0],  # final joint positions
]

time_steps = [0, 10, 30]  # time steps corresponding to the joint positions

joint_velocity_limits = [1, 1, 1, 1, 1, 1]  # velocity limits for the joints

trajectories = generate_trajectory(joint_positions, time_steps, joint_velocity_limits)

print(trajectories)
if trajectories is not None:
    for t in np.arange(0, 2, 0.01):
        joint_angles_t = [traj(t) for traj in trajectories]
        print(joint_angles_t)
