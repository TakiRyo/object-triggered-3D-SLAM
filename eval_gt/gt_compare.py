#3d model
import open3d as o3d
import numpy as np

# Define the path to your GT mesh file
MESH_FILE = "/home/ros2_env/taki/otslam/eval_gt/cardboard_box/meshes/cardboard_box.dae"
# Define the output path for the GT point cloud
GT_PCD_FILE = "/home/ros2_env/taki/otslam/3d_model/object_scan/3d_reconst/Object_0.ply" 

# 1. Load the mesh
try:
    gt_mesh = o3d.io.read_triangle_mesh(MESH_FILE)
    print(f"Loaded mesh with {len(np.asarray(gt_mesh.vertices))} vertices.")
except Exception as e:
    print(f"Error loading mesh: {e}")
    # You might need to check if the Open3D build supports DAE files. 
    # If not, use MeshLab to convert DAE -> OBJ/STL first.

# 2. Sample a dense point cloud from the mesh surface
# We sample 100,000 points to create a high-resolution Ground Truth reference.
NUMBER_OF_POINTS = 100000 
gt_pcd = gt_mesh.sample_points_uniformly(number_of_points=NUMBER_OF_POINTS)

# 3. Save the dense point cloud as PLY
o3d.io.write_point_cloud(GT_PCD_FILE, gt_pcd)
print(f"Successfully saved GT point cloud to {GT_PCD_FILE}")