from scipy.spatial.transform import Slerp
import numpy as np

# points1 = np.array([[2.20704160e-02, 7.63382043e-02, 7.96093532e-02, 1.31920308e-01],
#                                   [5.07765549e-04, 3.56598531e-02, 5.28265197e-02, 1.21474713e-01],
#                                   [3.14204778e-01, 4.41307716e-01, 3.08733099e-01, 3.00000012e-01]
#                                   ]).reshape(4,3)
# points2= np.array([[0.17676407, 0.19024703, 0.19957502, 0.21023896],
#                                       [0.1184915, 0.14825233, 0.1683707, 0.1904569],
#                                       [0.30486624, 0.26635709, 0.23439874, 0.18156405]
#                                       ]).reshape(4,3)

points1 = np.array([[2.20704160e-02, 7.63382043e-02, 7.96093532e-02, 1.31920308e-01],
                                  [5.07765549e-04, 3.56598531e-02, 5.28265197e-02, 1.21474713e-01],
                                  [3.14204778e-01, 4.41307716e-01, 3.08733099e-01, 3.00000012e-01],
                                  [1.00000000e+00, 1.00000000e+00, 1.00000000e+00, 1.00000000e+00]])
points2= np.array([[0.17676407, 0.19024703, 0.19957502, 0.21023896],
                                      [0.1184915, 0.14825233, 0.1683707, 0.1904569],
                                      [0.30486624, 0.26635709, 0.23439874, 0.18156405],
                                      [1., 1., 1., 1.]])
T = np.dot(points1, np.linalg.inv(points2))

# Print the resulting transformation matrix
print(T)
