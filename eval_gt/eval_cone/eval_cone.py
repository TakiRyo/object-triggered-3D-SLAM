#3d_model

# Dec. 1 result. 
# (3d_model) ros2_env@2024-NEDO-DENKI-1:~/taki/otslam/eval_gt/eval_cone$ python3 eval_cone.py 
# 📂 Loading SLAM: /home/ros2_env/taki/otslam/eval_gt/eval_cone/cone_slam.ply
# 📍 Centered SLAM data to (0,0,0) for easier alignment.
# 🔨 Loading GT: cone.stl
# 🔨 Loading GT: cone.stl
# ✅ Created Combined GT with 100000 points.

# 👀 Opening Visualization... (Check positions!)

# --- 📊 EVALUATION RESULTS (Combined) ---
# ✅ Accuracy (Mean Error): 2.48 cm
# ⚠️ Completeness (Mean Error): 4.18 cm

# 💾 Saving visualization to file...
# ✅ Saved merged result to: /home/ros2_env/taki/otslam/eval_gt/eval_cone/cone_evaluation_result.ply


import open3d as o3d
import trimesh
import numpy as np
import copy
import os

# ==========================================
# 🛠️ CONFIGURATION (設定)
# ==========================================

# 1. FILE PATHS
SLAM_FILE = "/home/ros2_env/taki/otslam/eval_gt/eval_cone/cone_slam.ply"
GT_BLUE_FILE = "/home/ros2_env/taki/otslam/eval_gt/eval_cone/cone_blue/meshes/cone.stl"
GT_RED_FILE  = "/home/ros2_env/taki/otslam/eval_gt/eval_cone/cone_red/meshes/cone.stl"
OUTPUT_FILENAME = "cone_evaluation_result.ply"

# 2. GAZEBO SCALING (Check your .world file!)
# Usually cones are uniformly scaled (e.g., 1.0 or 0.5)
UNIT_SCALE = 0.01     # Set to 0.001 if STL is in mm
SCALE_X = 1.0
SCALE_Y = 1.0
SCALE_Z = 1.0

COMMON_Z = -0.3

# 3. MANUAL ALIGNMENT (Separate for each Cone)
# You need to place Blue and Red where they belong in the SLAM map.

# --- BLUE CONE CONFIG ---
BLUE_ROT   = [0.0, 0.0, 0.0]   # Rot X, Y, Z
BLUE_TRANS = [0.5, 0.5, COMMON_Z]   # Move X, Y, Z (Meters)

# --- RED CONE CONFIG ---
RED_ROT    = [0.0, 0.0, 0.0]
RED_TRANS  = [-0.395, -0.36, COMMON_Z]   # Example: Maybe it's 1 meter away?

# ==========================================

def load_and_scale_gt(filename, color):
    """Loads STL, Scales it, and colors it."""
    print(f"🔨 Loading GT: {os.path.basename(filename)}")
    try:
        mesh = trimesh.load(filename, force='mesh')
    except Exception as e:
        print(f"Error: {e}")
        return None

    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(mesh.dump())

    # Trimesh -> Open3D
    mesh.export("temp.ply")
    gt_mesh = o3d.io.read_triangle_mesh("temp.ply")
    if os.path.exists("temp.ply"): os.remove("temp.ply")
    
    # Sample Points
    pcd = gt_mesh.sample_points_uniformly(number_of_points=50000)
    
    # Apply Scale
    points = np.asarray(pcd.points)
    points = points * UNIT_SCALE
    points[:, 0] *= SCALE_X
    points[:, 1] *= SCALE_Y
    points[:, 2] *= SCALE_Z
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Paint color for visualization
    pcd.paint_uniform_color(color)
    return pcd

def get_rot_matrix(r_list):
    rx, ry, rz = np.radians(r_list[0]), np.radians(r_list[1]), np.radians(r_list[2])
    return o3d.geometry.get_rotation_matrix_from_xyz((rx, ry, rz))

def main():
    # 1. Load SLAM
    print(f"📂 Loading SLAM: {SLAM_FILE}")
    slam_pcd = o3d.io.read_point_cloud(SLAM_FILE)
    
    # Center SLAM roughly to 0,0,0 so we aren't working with huge coordinates
    slam_center = slam_pcd.get_center()
    slam_pcd.translate(-slam_center)
    print("📍 Centered SLAM data to (0,0,0) for easier alignment.")

    # 2. Load & Scale GTs
    gt_blue = load_and_scale_gt(GT_BLUE_FILE, [0, 0, 1]) # Blue
    gt_red  = load_and_scale_gt(GT_RED_FILE,  [1, 0, 0]) # Red

    # 3. Apply Manual Transforms (Relative to SLAM center)
    
    # Transform Blue
    R_blue = get_rot_matrix(BLUE_ROT)
    gt_blue.rotate(R_blue, center=(0,0,0))
    gt_blue.translate(BLUE_TRANS)

    # Transform Red
    R_red = get_rot_matrix(RED_ROT)
    gt_red.rotate(R_red, center=(0,0,0))
    gt_red.translate(RED_TRANS)

    # 4. Merge GTs (Create the "Master Ground Truth")
    gt_combined = gt_blue + gt_red
    print(f"✅ Created Combined GT with {len(gt_combined.points)} points.")

    # 5. Visual Check
    print("\n👀 Opening Visualization... (Check positions!)")
    # Draw Axes to help with X/Y/Z direction
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
    
    slam_pcd.paint_uniform_color([1, 0.706, 0]) # Yellow
    o3d.visualization.draw_geometries([slam_pcd, gt_blue, gt_red, axes], 
                                      window_name="Check Alignment")

    # 6. Evaluation
    print("\n--- 📊 EVALUATION RESULTS (Combined) ---")
    
    # Accuracy (SLAM -> Nearest GT Cone)
    dists_s2g = slam_pcd.compute_point_cloud_distance(gt_combined)
    accuracy = np.mean(dists_s2g)
    print(f"✅ Accuracy (Mean Error): {accuracy*100:.2f} cm")
    
    # Completeness (Combined GT -> SLAM)
    dists_g2s = gt_combined.compute_point_cloud_distance(slam_pcd)
    completeness = np.mean(dists_g2s)
    print(f"⚠️ Completeness (Mean Error): {completeness*100:.2f} cm")

    # 7. SAVE RESULT (New!)
    print("\n💾 Saving visualization to file...")
    
    # Merge SLAM + Blue GT + Red GT into one point cloud
    # (In Open3D, '+' concatenates point clouds)
    final_merged_pcd = slam_pcd + gt_blue + gt_red
    
    o3d.io.write_point_cloud(OUTPUT_FILENAME, final_merged_pcd)
    print(f"✅ Saved merged result to: {os.path.abspath(OUTPUT_FILENAME)}")

if __name__ == "__main__":
    main()