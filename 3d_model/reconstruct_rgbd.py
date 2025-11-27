# ## 3d_model
# ## construct 3d models for ALL objects found in the directory

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

# --- COORDINATE FIX MATRIX ---
# Converts ROS "Body" Frame (X-Forward, Z-Up) -> Open3D "Camera" Frame (Z-Forward, Y-Down)
T_fix = np.array([
    [0, -1,  0, 0],
    [0,  0, -1, 0],
    [1,  0,  0, 0],
    [0,  0,  0, 1]
])

def get_unique_object_names():
    """
    Scans the color directory and finds unique object prefixes.
    Example: Finds 'Object_0' from 'Object_0_1.jpg', 'Object_0_2.jpg'
    """
    all_files = glob.glob(os.path.join(color_dir, "*.jpg"))
    unique_names = set()

    for f in all_files:
        filename = os.path.basename(f) # e.g., "Object_0_1.jpg"
        
        # Split by underscore to separate Label, ID, and Frame Number
        # We assume the format is Name_ID_FrameNumber.jpg
        # We want to keep everything EXCEPT the last number.
        parts = filename.split('_')
        
        if len(parts) >= 2:
            # Rejoin everything except the last part (the frame number)
            # ["Object", "0", "1.jpg"] -> "Object_0"
            obj_name = "_".join(parts[:-1])
            unique_names.add(obj_name)
    
    return sorted(list(unique_names))

def reconstruct_object(obj_name):
    print(f"\n========================================")
    print(f"🛠️  Processing: {obj_name}")
    print(f"========================================")

    # 1. LOAD FILES SPECIFIC TO THIS OBJECT
    # Note: We use f"{obj_name}_*.jpg" to only get files for this object
    color_files = sorted(glob.glob(os.path.join(color_dir, f"{obj_name}_*.jpg")))
    depth_files = sorted(glob.glob(os.path.join(depth_dir, f"{obj_name}_*.png")))
    pose_files  = sorted(glob.glob(os.path.join(pose_dir, f"{obj_name}_*.txt")))
    
    n_frames = len(color_files)
    if n_frames == 0:
        print(f"❌ Error: No files found for {obj_name}")
        return

    print(f"   Found {n_frames} frames.")

    # 2. INITIALIZE VOLUME
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=0.01,  # 1cm voxel size
        sdf_trunc=0.04,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
    )

    # 3. INTEGRATION LOOP
    for i in range(n_frames):
        # Load Data
        color = o3d.io.read_image(color_files[i])
        depth = o3d.io.read_image(depth_files[i])
        pose_ros = np.loadtxt(pose_files[i])

        # Apply rotation to match Open3D standards
        pose_optical = pose_ros @ T_fix

        # Invert Matrix (TSDF requires Extrinsics: World-to-Camera)
        extrinsic = np.linalg.inv(pose_optical)

        # Create RGBD
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, 
            depth_scale=1000.0, # Uint16 (mm) -> Meters
            depth_trunc=3.0,    # Ignore background > 3m
            convert_rgb_to_intensity=False
        )

        # Integrate
        volume.integrate(rgbd, intrinsics, extrinsic)
        sys.stdout.write(f"\r   Integrate: {i+1}/{n_frames}")
        sys.stdout.flush()

    print("\n   Extracting mesh...")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    # 4. SAVE OUTPUT
    output_filename = f"{obj_name}.ply"
    output_path = os.path.join(save_dir, output_filename)
    o3d.io.write_triangle_mesh(output_path, mesh)
    print(f"✅ Saved: {output_path}")

def main():
    # 1. Find all objects
    objects = get_unique_object_names()
    
    if not objects:
        print("No objects found in directory!")
        return

    print(f"Found {len(objects)} objects: {objects}")

    # 2. Loop through each object and reconstruct
    for obj in objects:
        reconstruct_object(obj)

    print("\n🎉 All reconstructions finished!")

if __name__ == "__main__":
    main()