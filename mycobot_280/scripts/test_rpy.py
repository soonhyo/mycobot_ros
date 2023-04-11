import numpy as np

def rotation_matrix_to_rpy(R):
    sy = np.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

# Example usage
R = np.array([[0.93629336, -0.27509585, 0.21835066],
              [0.28962948, 0.95642509, -0.03695701],
              [-0.19866933, 0.0978434, 0.97517033]])
rpy = rotation_matrix_to_rpy(R)
print(rpy)
