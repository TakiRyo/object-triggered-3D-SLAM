#visualize depth

import cv2
import numpy as np

input_path  = "/home/ros2_env/taki/scannet_project/exported/depth/4000.png"
output_path = "/home/ros2_env/taki/scannet_project/amc/depth_visual/4000.png"

# Load depth (should be 16-bit PNG from ScanNet)
depth = cv2.imread(input_path, cv2.IMREAD_UNCHANGED)

if depth is None:
    raise RuntimeError("Failed to load depth image!")

# Convert from millimeters to meters
depth_m = depth.astype(np.float32) / 1000.0

# Remove invalid depth (0 or too far)
valid = (depth_m > 0) & (depth_m < 5.0)
depth_clean = np.zeros_like(depth_m)
depth_clean[valid] = depth_m[valid]

# Inverse depth (far = bright)
depth_inv = depth_clean.copy()
depth_inv[depth_inv == 0] = 5.0  # avoid division by zero
depth_inv = 1.0 / depth_inv

# Normalize to 0–255
depth_norm = cv2.normalize(depth_inv, None, 0, 255, cv2.NORM_MINMAX)
depth_vis = depth_norm.astype(np.uint8)

# Save
cv2.imwrite(output_path, depth_vis)

print("Saved:", output_path)
