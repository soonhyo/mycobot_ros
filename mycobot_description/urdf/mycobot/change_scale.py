import pymeshlab as ml
import os

# Set the directory containing your .obj files
directory = 'mesh/gripper'

# Set the scaling factor
scaling_factor = 0.001

# Iterate over all .obj files in the directory
for filename in os.listdir(directory):
    if filename.endswith('.dae'):
        # Load the mesh
        ms = ml.MeshSet()
        full_path = os.path.join(directory, filename)
        ms.load_new_mesh(full_path)

        # Scale the mesh
        ms.compute_matrix_from_scaling_or_normalization(axisx=scaling_factor, axisy=scaling_factor, axisz=scaling_factor)
        # Save the scaled mesh as .obj
        output_filename = os.path.splitext(filename)[0] + '.obj'
        output_path = os.path.join(directory, output_filename)
        ms.save_current_mesh(output_path)
