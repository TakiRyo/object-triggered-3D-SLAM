# ## 3d_model_filtered.py
# ## Construct 3d models based on manual FRAME RANGES

import open3d as o3d
import numpy as np
import os
import sys

# --- CONFIGURATION ---
base_dir = "/home/ros2_env/taki/otslam/3d_model/object_scan_2"
color_dir = os.path.join(base_dir, "color")
depth_dir = os.path.join(base_dir, "depth")
pose_dir  = os.path.join(base_dir, "poses")
save_dir  = os.path.join(base_dir, "3d_reconst")

if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# --- MANUAL OBJECT RANGES ---
# Define your objects here.
# Format: "Output_Name": (Start_Frame_Number, End_Frame_Number)
# The code will look for files like "Object_0_{i}.jpg"
FILE_PREFIX = "Object_0"

OBJECT_RANGES = {
    # Name used for .ply file   : (Start, End) - Inclusive
    "object_0"                  : (1, 18),
    "object_1"                  : (19, 36),
    "object_2"                  : (37, 54),
    # Add more as needed:
    # "object_3"                : (55, 70),
}

# --- Z-AXIS FILTERING ---
Z_FILTER_THRESHOLD = 0.03

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

def reconstruct_range(obj_name, start_frame, end_frame):
    print(f"\n========================================")
    print(f"🛠️  Processing: {obj_name} (Frames {start_frame} -> {end_frame})")
    print(f"========================================")

    # 1. INITIALIZE VOLUME
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=0.01,
        sdf_trunc=0.04,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
    )

    frames_processed = 0

    # 2. INTEGRATION LOOP (Manual Range)
    for i in range(start_frame, end_frame + 1):
        # Construct exact filenames based on ID
        # e.g., Object_0_1.jpg, Object_0_2.jpg
        c_name = f"{FILE_PREFIX}_{i}.jpg"
        d_name = f"{FILE_PREFIX}_{i}.png"
        p_name = f"{FILE_PREFIX}_{i}.txt"

        c_path = os.path.join(color_dir, c_name)
        d_path = os.path.join(depth_dir, d_name)
        p_path = os.path.join(pose_dir, p_name)

        # Check if file exists
        if not os.path.exists(c_path):
            print(f"   ⚠️ Warning: File missing {c_name}, skipping...")
            continue

        try:
            # Load Data
            color = o3d.io.read_image(c_path)
            depth = o3d.io.read_image(d_path)
            pose_ros = np.loadtxt(p_path)

            pose_optical = pose_ros @ T_fix
            extrinsic = np.linalg.inv(pose_optical)

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color, depth, 
                depth_scale=1000.0,
                depth_trunc=3.0,
                convert_rgb_to_intensity=False
            )

            volume.integrate(rgbd, intrinsics, extrinsic)
            sys.stdout.write(f"\r   Integrate: Frame {i}")
            sys.stdout.flush()
            frames_processed += 1
        except Exception as e:
            print(f"\n   ⚠️ Error on frame {i}: {e}")

    if frames_processed == 0:
        print(f"\n❌ No frames were integrated for {obj_name}. Check your ranges or file paths.")
        return

    print("\n   Extracting mesh...")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    if len(mesh.vertices) == 0:
        print("❌ Warning: Mesh is empty! Check poses or depth scale.")
        return

    # --- 3. Z-FILTERING STEP ---
    print(f"   Filtering points below Z < {Z_FILTER_THRESHOLD:.2f}m...")
    
    pcd = mesh.sample_points_uniformly(number_of_points=100000)
    
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    mask = points[:, 2] >= Z_FILTER_THRESHOLD
    
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
    filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
    
    print(f"   Points remaining: {len(filtered_pcd.points)}")

    # 4. SAVE OUTPUT
    output_filename = f"{obj_name}.ply"
    output_path = os.path.join(save_dir, output_filename)
    
    o3d.io.write_point_cloud(output_path, filtered_pcd)
    print(f"✅ Saved 3D Model: {output_path}")

def main():
    print(f"Starting reconstruction for {len(OBJECT_RANGES)} objects...")
    
    for name, (start, end) in OBJECT_RANGES.items():
        reconstruct_range(name, start, end)

    print("\n🎉 All reconstructions finished!")

if __name__ == "__main__":
    main()