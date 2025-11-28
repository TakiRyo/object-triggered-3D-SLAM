# ## hybrid_map_generator.py
# ## Combines 2D PGM Map + Multiple 3D PLY Objects
# 3d model 

import yaml
import cv2
import numpy as np
import open3d as o3d
import os
import glob
import sys

# --- CONFIGURATION ---
# 2d map
map_base = "/home/ros2_env/taki/otslam/2d_map"
yaml_path = os.path.join(map_base, "map_nov27.yaml")
pgm_path  = os.path.join(map_base, "map_nov27.pgm")

# 3d map directory (Where Object_0.ply, Object_1.ply are located)
obj_dir  = "/home/ros2_env/taki/otslam/3d_model/object_scan_2/3d_reconst"

# Output
save_path = "/home/ros2_env/taki/otslam/fusion/hybrid_maps/hybrid_map.ply"

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
    # Filter: Occupied pixels (Black < 100)
    rows, cols = np.where(img < 100)

    points = []
    print(f"   -> Generating 3D Walls from {len(rows)} pixels...")

    for r, c in zip(rows, cols):
        wx = ox + (c * res)
        wy = oy + ((h - 1 - r) * res)
        
        # Z=0.0 for floor map
        points.append([wx, wy, 0.0])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.paint_uniform_color([0.2, 0.2, 0.2]) # Dark Gray Walls
    return pcd

def load_all_objects(directory):
    # Find ALL .ply files
    ply_files = sorted(glob.glob(os.path.join(directory, "*.ply")))
    
    if len(ply_files) == 0:
        print(f"❌ Error: No .ply files found in {directory}")
        return None
    
    print(f"   -> Found {len(ply_files)} objects: {[os.path.basename(f) for f in ply_files]}")

    # Create an empty point cloud to hold all objects
    combined_objects = o3d.geometry.PointCloud()
    
    for f in ply_files:
        print(f"      Loading: {os.path.basename(f)}...")
        try:
            # Load
            temp_pcd = o3d.io.read_point_cloud(f)
            
            # Convert Mesh -> PCD if necessary
            if len(temp_pcd.points) == 0:
                mesh = o3d.io.read_triangle_mesh(f)
                temp_pcd = mesh.sample_points_uniformly(number_of_points=15000)
            
            # Color them RED (so they stand out against the gray map)
            # Optional: You could give random colors here if you want
            temp_pcd.paint_uniform_color([1.0, 0.0, 0.0]) 

            # Merge into the main cloud
            combined_objects += temp_pcd
            
        except Exception as e:
            print(f"❌ Error loading {f}: {e}")

    return combined_objects

def main():
    print("--- 1. Processing Global Map ---")
    map_pcd = create_map_cloud(yaml_path, pgm_path)
    if map_pcd is None: return

    print("\n--- 2. Processing Local Objects ---")
    all_objs_pcd = load_all_objects(obj_dir)
    
    if all_objs_pcd is None or len(all_objs_pcd.points) == 0:
        print(" CRITICAL WARNING: No objects loaded.")
        print("    Continuing with Map Only...")
        combined_pcd = map_pcd
    else:
        print(f"   -> Total object points: {len(all_objs_pcd.points)}")
        
        # 3. MERGE
        print("\n--- 3. Merging & Saving ---")
        combined_pcd = map_pcd + all_objs_pcd

    # Save
    if not os.path.exists(os.path.dirname(save_path)):
        os.makedirs(os.path.dirname(save_path))
        
    o3d.io.write_point_cloud(save_path, combined_pcd)
    
    print(f"✅ SUCCESS! Hybrid map saved to:")
    print(f"   {save_path}")

    # 4. VISUALIZE
    print("\nOpening Visualizer... (Gray=Map, Red=Objects)")
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    o3d.visualization.draw_geometries([combined_pcd, origin], window_name="Hybrid Map")

if __name__ == "__main__":
    main()