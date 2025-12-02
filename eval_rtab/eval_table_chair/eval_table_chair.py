# (3d_model) ros2_env@2024-NEDO-DENKI-1:~/taki/otslam/eval_gt/eval_table_chair$ python3 eval_table_chair.py
# 📂 Loading SLAM: /home/ros2_env/taki/otslam/eval_gt/eval_table_chair/table_chair_slam.ply
# 📍 Centered SLAM data to (0,0,0).
# 🔨 Loading GT: table_lightmap.dae
# 🔨 Loading GT: Chair.obj
# 🔄 Placing 1 Table and 4 Chairs...

# 👀 Opening Visualization... (Adjust TRANS/ROT in config)

# --- 📊 EVALUATION RESULTS ---
# ✅ Accuracy (Mean Error): 2.62 cm
# ⚠️ Completeness (Mean Error): 8.58 cm

# 💾 Saving result...
# ✅ Saved to table_chair_eval_result.ply
# (3d_model) ros2_env@2024-NEDO-DENKI-1:~/taki/otslam/eval_gt/eval_table_chair


import open3d as o3d
import trimesh
import numpy as np
import copy
import os

# ==========================================
# 🛠️ CONFIGURATION (設定)
# ==========================================

# 1. FILE PATHS
# Adjust this to your actual SLAM file name
SLAM_FILE = "/home/ros2_env/taki/otslam/eval_gt/eval_table_chair/table_chair_slam.ply"
TABLE_FILE = "/home/ros2_env/taki/otslam/eval_gt/eval_table_chair/table_marble/meshes/table_lightmap.dae"
CHAIR_FILE = "/home/ros2_env/taki/otslam/eval_gt/eval_table_chair/Chair/meshes/Chair.obj"

# 2. SCALING (Check your world file for exact numbers!)
UNIT_SCALE = 1.0  # Use 0.001 if meshes are in mm

# --- TABLE SCALE ---
# <scale>...</scale> in Gazebo world
TABLE_SCALE = [0.258, 0.258, 0.258] 

# --- CHAIR SCALE ---
# <scale>...</scale> in Gazebo world
CHAIR_SCALE = [0.0075, 0.0075, 0.0075]

# 3. MANUAL ALIGNMENT (5 Objects)
# Adjust these to snap the GT onto the Yellow SLAM cloud.

# Common Z (Height adjustment for all objects)
COMMON_Z = -0.6
# [A] TABLE (Center)
TABLE_ROT   = [0.0, 0.0, -1.0]  
TABLE_TRANS = [0.035, 0.17, 0.0]

# red arrow (x) is right, green arrow (y) is up. blue arrow (z) is toward to me 

# [B] CHAIR 1 (top right)
CHAIR_1_ROT   = [0.0, 0.0, 0.0]
CHAIR_1_TRANS = [0.6, 1.45, COMMON_Z]

# [C] CHAIR 2 (top left)
CHAIR_2_ROT   = [0.0, 0.0, 0.0] # Facing opposite
CHAIR_2_TRANS = [-0.45, 1.45, COMMON_Z]

# [D] CHAIR 3 (bottom left)
CHAIR_3_ROT   = [0.0, 0.0, 180.0]
CHAIR_3_TRANS = [-0.6, -1.25, COMMON_Z]

# [E] CHAIR 4 (bottom right)
CHAIR_4_ROT   = [0.0, 0.0, 180.0]
CHAIR_4_TRANS = [0.6, -1.25, COMMON_Z]

# ==========================================

def load_and_scale_gt(filename, scale_factors, color):
    """Loads mesh (DAE/OBJ), scales it, and samples points."""
    print(f"🔨 Loading GT: {os.path.basename(filename)}")
    try:
        mesh = trimesh.load(filename, force='mesh')
    except Exception as e:
        print(f"Error loading mesh: {e}")
        return None

    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(mesh.dump())

    # Export/Import to Open3D
    mesh.export("temp_obj.ply")
    gt_mesh = o3d.io.read_triangle_mesh("temp_obj.ply")
    if os.path.exists("temp_obj.ply"): os.remove("temp_obj.ply")
    
    # Sample
    pcd = gt_mesh.sample_points_uniformly(number_of_points=50000)
    
    # Apply Scale
    points = np.asarray(pcd.points)
    points = points * UNIT_SCALE
    points[:, 0] *= scale_factors[0]
    points[:, 1] *= scale_factors[1]
    points[:, 2] *= scale_factors[2]
    pcd.points = o3d.utility.Vector3dVector(points)
    
    pcd.paint_uniform_color(color)
    return pcd

def apply_transform(pcd, rot, trans):
    """Helper to apply rotation and translation."""
    # Convert degrees to radians
    rx, ry, rz = np.radians(rot[0]), np.radians(rot[1]), np.radians(rot[2])
    R = o3d.geometry.get_rotation_matrix_from_xyz((rx, ry, rz))
    
    # We copy so we don't mess up the original if reused
    pcd_copy = copy.deepcopy(pcd)
    pcd_copy.rotate(R, center=(0,0,0))
    pcd_copy.translate(trans)
    return pcd_copy

def main():
    # 1. Load SLAM
    print(f"📂 Loading SLAM: {SLAM_FILE}")
    try:
        slam_pcd = o3d.io.read_point_cloud(SLAM_FILE)
    except:
        print("⚠️ Failed to load SLAM file. Check path!")
        return

    # Center SLAM for easier manual alignment
    slam_center = slam_pcd.get_center()
    slam_pcd.translate(-slam_center)
    print("📍 Centered SLAM data to (0,0,0).")

    # 2. Load Base GT Models
    # Table = Blue, Chairs = Red
    gt_table_base = load_and_scale_gt(TABLE_FILE, TABLE_SCALE, [0, 0.65, 0.93]) 
    gt_chair_base = load_and_scale_gt(CHAIR_FILE, CHAIR_SCALE, [1, 0, 0])

    if gt_table_base is None or gt_chair_base is None: return

    # 3. Create 5 Instances & Position Them
    print("🔄 Placing 1 Table and 4 Chairs...")
    
    gt_table   = apply_transform(gt_table_base, TABLE_ROT,   TABLE_TRANS)
    gt_chair_1 = apply_transform(gt_chair_base, CHAIR_1_ROT, CHAIR_1_TRANS)
    gt_chair_2 = apply_transform(gt_chair_base, CHAIR_2_ROT, CHAIR_2_TRANS)
    gt_chair_3 = apply_transform(gt_chair_base, CHAIR_3_ROT, CHAIR_3_TRANS)
    gt_chair_4 = apply_transform(gt_chair_base, CHAIR_4_ROT, CHAIR_4_TRANS)

    # 4. Merge Everything
    gt_combined = gt_table + gt_chair_1 + gt_chair_2 + gt_chair_3 + gt_chair_4

    # 5. Visual Check
    print("\n👀 Opening Visualization... (Adjust TRANS/ROT in config)")
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
    slam_pcd.paint_uniform_color([1, 0.706, 0]) # Yellow
    
    o3d.visualization.draw_geometries([slam_pcd, gt_combined, axes], 
                                      window_name="Check Table & Chairs")

    # 6. Evaluation
    print("\n--- 📊 EVALUATION RESULTS ---")
    dists_s2g = slam_pcd.compute_point_cloud_distance(gt_combined)
    accuracy = np.mean(dists_s2g)
    print(f"✅ Accuracy (Mean Error): {accuracy*100:.2f} cm")
    
    dists_g2s = gt_combined.compute_point_cloud_distance(slam_pcd)
    completeness = np.mean(dists_g2s)
    print(f"⚠️ Completeness (Mean Error): {completeness*100:.2f} cm")

    # 7. Save
    print("\n💾 Saving result...")
    final_merged_pcd = slam_pcd + gt_combined
    o3d.io.write_point_cloud("table_chair_eval_result.ply", final_merged_pcd)
    print("✅ Saved to table_chair_eval_result.ply")

if __name__ == "__main__":
    main()