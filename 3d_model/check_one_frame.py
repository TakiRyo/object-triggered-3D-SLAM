## 3d_model
## construct 3d model from one rgbd. 

import open3d as o3d
import os

base_dir = "/home/ros2_env/taki/otslam/3d_model/object_scan"
color_path = os.path.join(base_dir, "color/color_0000.png")
depth_path = os.path.join(base_dir, "depth/depth_0000.png")

# Intrinsics
fx, fy = 565.6009, 565.6009
cx, cy = 320.5, 240.5
width, height = 640, 480
intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

# ⚠️ Adjust this depending on your depth format
depth_scale = 1000.0   # if depth saved as uint16 in millimeters

color_raw = o3d.io.read_image(color_path)
depth_raw = o3d.io.read_image(depth_path)
rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw,
    depth_scale=depth_scale, depth_trunc=5.0, convert_rgb_to_intensity=False
)

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)
pcd = pcd.voxel_down_sample(0.01)

o3d.visualization.draw_geometries([pcd])
print("✅ Displayed single-frame point cloud.")
