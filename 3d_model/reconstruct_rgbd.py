import open3d as o3d
import numpy as np
import yaml
import os

# --- user config ---
base_dir = "/home/ros2_env/taki/otslam/ros2_ws/src/rgbd_capture/object_scan"
color_dir = os.path.join(base_dir, "color")
depth_dir = os.path.join(base_dir, "depth")
pose_dir  = os.path.join(base_dir, "poses")

# Camera intrinsics (same for RGB & depth)
fx, fy = 565.6009, 565.6009
cx, cy = 320.5, 240.5
width, height = 640, 480
intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

# --- helper: load pose (4x4) ---
def load_pose(path):
    return np.loadtxt(path).reshape(4,4)

# --- frames to use ---
frames = ["0001", "0002", "0003", "0004"]

pcds = []
for fid in frames:
    color_path = os.path.join(color_dir, f"color_{fid}.png")
    depth_path = os.path.join(depth_dir, f"depth_{fid}.png")
    pose_path  = os.path.join(pose_dir,  f"pose_{fid}.txt")

    print(f"Loading frame {fid}...")
    color_raw = o3d.io.read_image(color_path)
    depth_raw = o3d.io.read_image(depth_path)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw,
        depth_scale=0.01, depth_trunc=5.0, convert_rgb_to_intensity=False
    )

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)
    T = load_pose(pose_path)
    pcd.transform(T)
    pcds.append(pcd)

# --- merge & downsample ---
pcd_combined = o3d.geometry.PointCloud()
for p in pcds:
    pcd_combined += p
pcd_combined = pcd_combined.voxel_down_sample(voxel_size=0.01)

# --- save & visualize ---
o3d.io.write_point_cloud(os.path.join(base_dir, "combined_pointcloud.ply"), pcd_combined)
print("✅ Saved combined_pointcloud.ply")

o3d.visualization.draw_geometries([pcd_combined])
