import open3d as o3d
import trimesh
import numpy as np
import copy
import os

# ==========================================
# 🛠️ CONFIGURATION
# ==========================================

# 1. FILE PATHS
# CHANGE THIS to your RTAB-Map (Baseline) file
SLAM_FILE = "/home/ros2_env/taki/otslam/eval_rtab/eval_table_chair/rtab_table_chair.ply"

# Keep these the same as before
TABLE_FILE = "/home/ros2_env/taki/otslam/eval_gt/eval_table_chair/table_marble/meshes/table_lightmap.dae"
CHAIR_FILE = "/home/ros2_env/taki/otslam/eval_gt/eval_table_chair/Chair/meshes/Chair.obj"

# 2. SCALING (Same as before)
UNIT_SCALE = 1.0
TABLE_SCALE = [0.258, 0.258, 0.258] 
CHAIR_SCALE = [0.0075, 0.0075, 0.0075]

# 3. MANUAL ALIGNMENT (Reuse your previous settings!)
# If RTAB-Map drifted differently, tweak these slightly.
COMMON_Z = -0.6
TABLE_ROT   = [0.0, 0.0, -1.0]  
TABLE_TRANS = [0.035, 0.17, 0.0]

CHAIR_1_ROT   = [0.0, 0.0, 0.0];    CHAIR_1_TRANS = [0.6, 1.45, COMMON_Z]
CHAIR_2_ROT   = [0.0, 0.0, 0.0];    CHAIR_2_TRANS = [-0.45, 1.45, COMMON_Z]
CHAIR_3_ROT   = [0.0, 0.0, 180.0];  CHAIR_3_TRANS = [-0.6, -1.25, COMMON_Z]
CHAIR_4_ROT   = [0.0, 0.0, 180.0];  CHAIR_4_TRANS = [0.6, -1.25, COMMON_Z]

# ==========================================

def load_and_scale_gt(filename, scale_factors, color):
    try: mesh = trimesh.load(filename, force='mesh')
    except: return None
    if isinstance(mesh, trimesh.Scene): mesh = trimesh.util.concatenate(mesh.dump())
    mesh.export("temp_obj.ply")
    gt_mesh = o3d.io.read_triangle_mesh("temp_obj.ply")
    if os.path.exists("temp_obj.ply"): os.remove("temp_obj.ply")
    pcd = gt_mesh.sample_points_uniformly(number_of_points=50000)
    points = np.asarray(pcd.points) * UNIT_SCALE
    points[:, 0] *= scale_factors[0]; points[:, 1] *= scale_factors[1]; points[:, 2] *= scale_factors[2]
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.paint_uniform_color(color)
    return pcd

def apply_transform(pcd, rot, trans):
    rx, ry, rz = np.radians(rot[0]), np.radians(rot[1]), np.radians(rot[2])
    R = o3d.geometry.get_rotation_matrix_from_xyz((rx, ry, rz))
    pcd_copy = copy.deepcopy(pcd)
    pcd_copy.rotate(R, center=(0,0,0))
    pcd_copy.translate(trans)
    return pcd_copy

def main():
    print(f"📂 Loading Baseline SLAM: {SLAM_FILE}")
    try: slam_pcd = o3d.io.read_point_cloud(SLAM_FILE)
    except: return

    slam_center = slam_pcd.get_center()
    slam_pcd.translate(-slam_center)
    print("📍 Centered SLAM data.")

    gt_table_base = load_and_scale_gt(TABLE_FILE, TABLE_SCALE, [0, 0.65, 0.93]) 
    gt_chair_base = load_and_scale_gt(CHAIR_FILE, CHAIR_SCALE, [1, 0, 0])

    print("🔄 Placing Objects...")
    gt_combined = apply_transform(gt_table_base, TABLE_ROT, TABLE_TRANS) + \
                  apply_transform(gt_chair_base, CHAIR_1_ROT, CHAIR_1_TRANS) + \
                  apply_transform(gt_chair_base, CHAIR_2_ROT, CHAIR_2_TRANS) + \
                  apply_transform(gt_chair_base, CHAIR_3_ROT, CHAIR_3_TRANS) + \
                  apply_transform(gt_chair_base, CHAIR_4_ROT, CHAIR_4_TRANS)

    print("\n👀 Visual Check: Blue/Red is GT, Yellow is Baseline.")
    slam_pcd.paint_uniform_color([1, 0.706, 0]) # Yellow
    o3d.visualization.draw_geometries([slam_pcd, gt_combined], window_name="Baseline vs GT")

    print("\n--- 📊 BASELINE RESULTS ---")
    dists = slam_pcd.compute_point_cloud_distance(gt_combined)
    accuracy = np.mean(dists)
    rmse = np.sqrt(np.mean(np.asarray(dists) ** 2))
    print(f"✅ Accuracy (Mean Error): {accuracy*100:.2f} cm")
    print(f"✅ RMSE: {rmse*100:.2f} cm")

if __name__ == "__main__":
    main()