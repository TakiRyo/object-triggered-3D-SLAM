import open3d as o3d
import numpy as np

# Paths
slam_file = "/home/ros2_env/taki/otslam/eval_gt/Object_0.ply" # Your SLAM result
gt_file = "gt_cardboard_box.ply"   # Your new GT

def get_size(pcd):
    min_b = pcd.get_min_bound()
    max_b = pcd.get_max_bound()
    dims = max_b - min_b
    return dims

# Load
slam_pcd = o3d.io.read_point_cloud(slam_file)
gt_pcd = o3d.io.read_point_cloud(gt_file)

# Check Sizes
print(f"SLAM Size (X, Y, Z): {get_size(slam_pcd)}")
print(f"GT   Size (X, Y, Z): {get_size(gt_pcd)}")

# Check scale difference
slam_x = get_size(slam_pcd)[0]
gt_x = get_size(gt_pcd)[0]

if gt_x > slam_x * 10:
    print("\n⚠️ WARNING: GT is much bigger! Likely mm vs m scaling issue.")
elif gt_x < slam_x * 0.1:
    print("\n⚠️ WARNING: GT is much smaller!")
else:
    print("\n✅ Scale looks correct. High point count is good!")