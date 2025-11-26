#3d model

import yaml
import cv2
import numpy as np
import open3d as o3d
import os
import glob
import sys

# --- CONFIGURATION ---
#2d map
map_base = "/home/ros2_env/taki/otslam/2d_map"
yaml_path = os.path.join(map_base, "2d_map.yaml")
pgm_path  = os.path.join(map_base, "2d_map.pgm")
#3d map
obj_dir  = "/home/ros2_env/taki/otslam/3d_model/object_scan_2/3d_reconst"
#hybrid map
save_path = "/home/ros2_env/taki/otslam/fusion/hybrid_maps/hybrid_map_2.ply"

def create_map_cloud(yaml_file, pgm_file):
    print(f"   -> Loading Map: {pgm_file}")
    if not os.path.exists(yaml_file):
        print(f"❌ Error: YAML not found: {yaml_file}")
        return None
    
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    
    res = data['resolution']
    origin = data['origin'] 
    ox, oy = origin[0], origin[1]

    img = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
    if img is None: 
        print(f"❌ Error: PGM not found: {pgm_file}")
        return None

    h, w = img.shape
    # Filter: Occupied pixels (Black)
    rows, cols = np.where(img < 100)

    points = []
    print(f"   -> Generating 3D Walls from {len(rows)} pixels...")

    for r, c in zip(rows, cols):
        wx = ox + (c * res)
        wy = oy + ((h - 1 - r) * res)
        
        # --- CHANGED HERE: No Z-axis extension ---
        # Just one point at z=0.0
        points.append([wx, wy, 0.0])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.paint_uniform_color([0.2, 0.2, 0.2]) # Dark Gray Walls
    return pcd

def load_object_pcd(directory):
    # Auto-find the .ply file
    ply_files = glob.glob(os.path.join(directory, "*.ply"))
    
    if len(ply_files) == 0:
        print(f"❌ Error: No .ply files found in {directory}")
        return None
    
    # Pick the most recently modified file
    target_file = max(ply_files, key=os.path.getmtime)
    print(f"   -> Found Object File: {target_file}")
    
    try:
        # Load as PointCloud
        pcd = o3d.io.read_point_cloud(target_file)
        if len(pcd.points) == 0:
            # Try loading as mesh and converting
            mesh = o3d.io.read_triangle_mesh(target_file)
            pcd = mesh.sample_points_uniformly(number_of_points=15000)
        
        # Color it RED so we can see it
        pcd.paint_uniform_color([1.0, 0.0, 0.0]) 
        return pcd
    except Exception as e:
        print(f"❌ Error loading object: {e}")
        return None

def main():
    print("--- 1. Processing Global Map ---")
    # yaml_path = os.path.join(map_base, "2d_map.yaml")
    # pgm_path  = os.path.join(map_base, "2d_map.pgm")
    
    map_pcd = create_map_cloud(yaml_path, pgm_path)
    if map_pcd is None: return

    print("\n--- 2. Processing Local Object ---")
    obj_pcd = load_object_pcd(obj_dir)
    
    if obj_pcd is None or len(obj_pcd.points) == 0:
        print(" CRITICAL WARNING: Object Cloud is EMPTY or MISSING.")
        print("    Stopping save. Please check if 'reconstruct_rgbd.py' ran correctly.")
        return

    print(f"   -> Object loaded with {len(obj_pcd.points)} points.")

    # 3. MERGE
    print("\n--- 3. Merging & Saving ---")
    combined_pcd = map_pcd + obj_pcd

    o3d.io.write_point_cloud(save_path, combined_pcd)
    
    print(f"✅ SUCCESS! Hybrid map saved to:")
    print(f"   {save_path}")

    # 4. VISUALIZE
    # Add coordinate frame for scale
    print("\nOpening Visualizer... (Look for the RED object inside the GRAY walls)")
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    o3d.visualization.draw_geometries([combined_pcd, origin], window_name="Hybrid Map")

if __name__ == "__main__":
    main()



