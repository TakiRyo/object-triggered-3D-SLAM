# ## 3d_model_filtered.py
# ## construct 3d models for ALL objects (Supports 4, 6, or any number of frames)

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

# --- Z-AXIS FILTERING ---
# Removes floor points (Z < 0.03m)
Z_FILTER_THRESHOLD = 0.03
# ------------------------

# --- CAMERA INTRINSICS ---
fx, fy = 565.6009, 565.6009
cx, cy = 320.5, 240.5
width, height = 640, 480
intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

# --- COORDINATE FIX MATRIX ---
T_fix = np.array([
    [0, -1,  0, 0],
    [0,  0, -1, 0],
    [1,  0,  0, 0],
    [0,  0,  0, 1]
])

def get_unique_object_names():
    """
    Scans the color directory and finds unique object prefixes.
    """
    all_files = glob.glob(os.path.join(color_dir, "*.jpg"))
    unique_names = set()

    for f in all_files:
        filename = os.path.basename(f)
        # Assuming format: Label_ID_Frame.jpg
        # Example: Object_0_1.jpg
        parts = filename.split('_')
        
        if len(parts) >= 2:
            # Join everything except the last part (the frame number)
            # ["Object", "0", "1.jpg"] -> "Object_0"
            obj_name = "_".join(parts[:-1])
            unique_names.add(obj_name)
    
    return sorted(list(unique_names))

def reconstruct_object(obj_name):
    print(f"\n========================================")
    print(f"🛠️  Processing: {obj_name}")
    print(f"========================================")

    # 1. LOAD FILES SPECIFIC TO THIS OBJECT
    # "glob" will find ALL files that start with {obj_name}_
    # So if you have Object_0_1...Object_0_6, it finds all 6.
    color_files = sorted(glob.glob(os.path.join(color_dir, f"{obj_name}_*.jpg")))
    depth_files = sorted(glob.glob(os.path.join(depth_dir, f"{obj_name}_*.png")))
    pose_files  = sorted(glob.glob(os.path.join(pose_dir, f"{obj_name}_*.txt")))
    
    n_frames = len(color_files)
    if n_frames == 0:
        print(f"❌ Error: No files found for {obj_name}")
        return

    # PRINT CONFIRMATION
    print(f"📸 Found {n_frames} images for reconstruction.")

    # 2. INITIALIZE VOLUME
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=0.01,  # 1cm voxel size
        sdf_trunc=0.04,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
    )

    # 3. INTEGRATION LOOP
    for i in range(n_frames):
        try:
            # Load Data
            color = o3d.io.read_image(color_files[i])
            depth = o3d.io.read_image(depth_files[i])
            pose_ros = np.loadtxt(pose_files[i])

            pose_optical = pose_ros @ T_fix
            extrinsic = np.linalg.inv(pose_optical)

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color, depth, 
                depth_scale=1000.0,
                depth_trunc=3.0,
                convert_rgb_to_intensity=False
            )

            volume.integrate(rgbd, intrinsics, extrinsic)
            sys.stdout.write(f"\r   Integrate: {i+1}/{n_frames}")
            sys.stdout.flush()
        except Exception as e:
            print(f"\n   ⚠️ Skipping frame {i+1} due to error: {e}")

    print("\n   Extracting mesh...")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    if len(mesh.vertices) == 0:
        print("❌ Warning: Mesh is empty! Check poses or depth scale.")
        return

    # --- 4. Z-FILTERING STEP ---
    print(f"   Filtering points below Z < {Z_FILTER_THRESHOLD:.2f}m...")
    
    # Convert mesh to point cloud
    pcd = mesh.sample_points_uniformly(number_of_points=100000)
    
    # Apply Mask
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    mask = points[:, 2] >= Z_FILTER_THRESHOLD
    
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
    filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
    
    print(f"   Points remaining: {len(filtered_pcd.points)}")

    # 5. SAVE OUTPUT
    output_filename = f"{obj_name}.ply"
    output_path = os.path.join(save_dir, output_filename)
    
    o3d.io.write_point_cloud(output_path, filtered_pcd)
    print(f"✅ Saved 3D Model: {output_path}")

def main():
    # 1. Find all objects
    objects = get_unique_object_names()
    
    if not objects:
        print("No objects found in directory!")
        return

    print(f"Found {len(objects)} unique objects: {objects}")

    # 2. Loop through each object and reconstruct
    for obj in objects:
        reconstruct_object(obj)

    print("\n🎉 All reconstructions finished!")

if __name__ == "__main__":
    main()