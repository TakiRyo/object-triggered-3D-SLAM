# ## 3d_model
# ## construct 3d model from 4 rgbds. 


import open3d as o3d
import numpy as np
import os
import glob
import sys

# --- CONFIGURATION ---
base_dir = "/home/ros2_env/taki/otslam/3d_model/object_scan_2"
color_dir = os.path.join(base_dir, "color")
depth_dir = os.path.join(base_dir, "depth")
pose_dir  = os.path.join(base_dir, "poses")
save_dir  = os.path.join(base_dir, "3d_reconst")

if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# --- CAMERA INTRINSICS ---
# Update these if your Gazebo settings are different
fx, fy = 565.6009, 565.6009
cx, cy = 320.5, 240.5
width, height = 640, 480
intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

def main():
    # 1. LOAD FILES (UPDATED to .png)
    # color_files = sorted(glob.glob(os.path.join(color_dir, "*.jpg"))) 
    color_files = sorted(glob.glob(os.path.join(color_dir, "Object_0_*.jpg")))
    depth_files = sorted(glob.glob(os.path.join(depth_dir, "Object_0_*.png")))
    pose_files  = sorted(glob.glob(os.path.join(pose_dir, "Object_0_*.txt")))

    n_frames = len(color_files)
    if n_frames == 0:
        print(f"❌ Error: No .png files found in {color_dir}")
        return

    print(f"✅ Found {n_frames} frames. Starting reconstruction...")

    # 2. INITIALIZE VOLUME
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=0.01,  # 1cm voxel size
        sdf_trunc=0.04,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
    )

    # 3. COORDINATE FIX MATRIX
    # Converts ROS "Body" Frame (X-Forward, Z-Up) -> Open3D "Camera" Frame (Z-Forward, Y-Down)
    # If your result is still weird, try changing this to np.eye(4)
    T_fix = np.array([
        [0, -1,  0, 0],
        [0,  0, -1, 0],
        [1,  0,  0, 0],
        [0,  0,  0, 1]
    ])

    # T_fix = np.eye(4)

    for i in range(n_frames):
        # Load Data
        color = o3d.io.read_image(color_files[i])
        depth = o3d.io.read_image(depth_files[i])
        pose_ros = np.loadtxt(pose_files[i])

        # --- STEP A: Fix Coordinate System ---
        # Apply rotation to match Open3D standards
        pose_optical = pose_ros @ T_fix

        # --- STEP B: Invert Matrix ---
        # TSDF requires World-to-Camera (Extrinsics), not Camera-to-World
        extrinsic = np.linalg.inv(pose_optical)

        # --- STEP C: Create RGBD ---
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, 
            depth_scale=1000.0, # Uint16 (mm) -> Meters
            depth_trunc=3.0,    # Ignore background > 3m
            convert_rgb_to_intensity=False
        )

        # --- STEP D: Integrate ---
        volume.integrate(rgbd, intrinsics, extrinsic)
        sys.stdout.write(f"\rProcessing frame {i+1}/{n_frames}...")
        sys.stdout.flush()

    print("\nExtracting mesh...")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    # Save output
    output_path = os.path.join(save_dir, "object_reconst.ply")
    o3d.io.write_triangle_mesh(output_path, mesh)
    print(f"✅ Success! Saved to: {output_path}")

    # Visualization
    # Add coordinate frame (Red=X, Green=Y, Blue=Z)
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
    o3d.visualization.draw_geometries([mesh, origin])

if __name__ == "__main__":
    main()


