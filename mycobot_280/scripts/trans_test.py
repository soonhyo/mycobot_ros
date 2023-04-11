import numpy as np

# Define the original coordinates and the corresponding target coordinates
orig_points = np.array([[1, 2, 3],
                        [4, 5, 6],
                        [7, 8, 9],
                        [10, 11, 12]])

target_points = np.array([[2, 3, 4],
                          [5, 6, 7],
                          [8, 9, 10],
                          [11, 12, 13]])

# Construct the homogeneous coordinates of the original points
orig_homogeneous = np.hstack((orig_points, np.ones((4, 1))))

# Calculate the transformation matrix using the method of least squares
transform_matrix, residuals, rank, singular_values = np.linalg.lstsq(orig_homogeneous, target_points, rcond=None)

# Print the transformation matrix
print("Transformation matrix:")
print(transform_matrix)
