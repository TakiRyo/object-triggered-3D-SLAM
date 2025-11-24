#3d_model

import open3d as o3d
import numpy as np
import glob
import os

# CONFIG
pose_dir = "/home/ros2_env/taki/otslam/3d_model/object_scan/poses"
pose_files = sorted(glob.glob(os.path.join(pose_dir, "*.txt")))

# YOUR FIX (We will test if this is correct)
T_fix = np.array([
    [0, -1,  0, 0],
    [0,  0, -1, 0],
    [1,  0,  0, 0],
    [0,  0,  0, 1]
])

vis_elements = []

# Create a coordinate frame for the World Origin (0,0,0)
world_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
vis_elements.append(world_origin)

print(f"Found {len(pose_files)} poses. Generating trajectory...")

for i, p_file in enumerate(pose_files):
    # 1. Load Raw ROS Pose
    pose_ros = np.loadtxt(p_file)
    
    # 2. Apply your T_fix
    pose_optical = pose_ros @ T_fix
    
    # 3. Create a small coordinate frame (Arrow) for the camera
    # Scale=0.2 meters
    camera_marker = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    
    # 4. Move the arrow to the camera pose
    # Note: We do NOT invert here. We want to see WHERE the camera is.
    camera_marker.transform(pose_optical)
    
    vis_elements.append(camera_marker)

# Visualize
print("Displaying Trajectory...")
print("Blue Arrow = Camera Look Direction")
print("Red Arrow = Camera Right")
o3d.visualization.draw_geometries(vis_elements)