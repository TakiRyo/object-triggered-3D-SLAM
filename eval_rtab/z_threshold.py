import open3d as o3d
import numpy as np

# Load
pcd = o3d.io.read_point_cloud("/home/ros2_env/taki/otslam/eval_rtab/mymap_whole_cloud.ply")

# Filter
points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)

# Mask: Z > 1.0 (Keep points ABOVE 1.0, effectively cutting off lower than 1)
# The user said "cut off lower than z= 1", so keep z >= 1.
mask = points[:, 2] >= -0.1

# Apply mask
cropped_pcd = pcd.select_by_index(np.where(mask)[0])

# Save
o3d.io.write_point_cloud("/home/ros2_env/taki/otslam/eval_rtab/output.ply", cropped_pcd)