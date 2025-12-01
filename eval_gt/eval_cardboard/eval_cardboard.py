#3d_model
import open3d as o3d
import trimesh
import numpy as np
import copy
import os

# ==========================================
# 🛠️ CONFIGURATION (設定)
# ==========================================

# 1. FILE PATHS
# Path to your SLAM result (ply)
SLAM_FILE = "/home/ros2_env/taki/otslam/eval_gt/eval_cardboard/cardboard_slam.ply"              
# Path to the raw Gazebo mesh (dae)
DAE_FILE = "/home/ros2_env/taki/otslam/eval_gt/eval_cardboard/cardboard_box/meshes/cardboard_box.dae"

# 2. GAZEBO SCALING (From your .world file)
# Fixes the "too big" or "wrong shape" issue
# <scale>1.25932 1.00745 0.755591</scale>
UNIT_SCALE = 0.001  # mm -> meters
# SCALE_X = 1.25932
# SCALE_Y = 1.00745
# SCALE_Z = 0.755591
SCALE_X = 1.62
SCALE_Y = 1.1
SCALE_Z = 0.50377

# 3. MANUAL ALIGNMENT (手動位置合わせ)
# Adjust these to snap the Yellow box onto the Blue box
# -----------------------------------------------------
# Rotation (Degrees)
ROT_X = 0.0   
ROT_Y = 0.0   
ROT_Z = 0.0   

# Translation (Meters) - Shift relative to center
# If Yellow is too high, decrease Z (e.g., -0.05)
TRANS_X = 0.05  
TRANS_Y = 0.0   
TRANS_Z = 0.0  # Try adjusting this to match the floor!
# -----------------------------------------------------

def generate_scaled_gt():
    """Loads DAE, converts to PCD, and applies Gazebo scaling."""
    print(f"🔨 Generating GT from: {DAE_FILE}")
    
    # 1. Load Mesh
    try:
        mesh = trimesh.load(DAE_FILE, force='mesh')
    except Exception as e:
        print(f"Error loading DAE: {e}")
        return None

    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(mesh.dump())

    # 2. Convert to Open3D
    mesh.export("temp_gt.ply")
    gt_mesh = o3d.io.read_triangle_mesh("temp_gt.ply")
    if os.path.exists("temp_gt.ply"): os.remove("temp_gt.ply")
    
    # 3. Sample Points
    gt_pcd = gt_mesh.sample_points_uniformly(number_of_points=100000)
    
    # 4. Apply Scaling (Non-Uniform Stretch)
    points = np.asarray(gt_pcd.points)
    
    # Convert mm -> m
    points = points * UNIT_SCALE
    
    # Apply Gazebo Stretch
    points[:, 0] *= SCALE_X
    points[:, 1] *= SCALE_Y
    points[:, 2] *= SCALE_Z
    
    gt_pcd.points = o3d.utility.Vector3dVector(points)
    
    print(f"✅ GT Generated. Size: {gt_pcd.get_max_bound() - gt_pcd.get_min_bound()}")
    return gt_pcd

def get_manual_rotation_matrix(rx, ry, rz):
    rx, ry, rz = np.radians(rx), np.radians(ry), np.radians(rz)
    R = o3d.geometry.get_rotation_matrix_from_xyz((rx, ry, rz))
    return R

def draw_registration_result(source, target, window_name="Result"):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])     # SLAM = Yellow
    target_temp.paint_uniform_color([0, 0.651, 0.929]) # GT = Blue
    
    # Axes: Red=X, Green=Y, Blue=Z
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0,0,0])
    
    o3d.visualization.draw_geometries([source_temp, target_temp, axes],
                                      window_name=window_name)

def main():
    # 1. Generate the Correct GT
    gt_pcd = generate_scaled_gt()
    if gt_pcd is None: return

    # 2. Load SLAM
    print(f"📂 Loading SLAM: {SLAM_FILE}")
    slam_pcd = o3d.io.read_point_cloud(SLAM_FILE)

    # 3. Centering (Bring both to 0,0,0)
    print("📍 Centering Clouds...")
    slam_pcd.translate(-slam_pcd.get_center())
    gt_pcd.translate(-gt_pcd.get_center())

    # 4. Apply Manual Transform
    print(f"🔄 Applying Manual Transform: Rot={ROT_X},{ROT_Y},{ROT_Z} | Trans={TRANS_X},{TRANS_Y},{TRANS_Z}")
    R = get_manual_rotation_matrix(ROT_X, ROT_Y, ROT_Z)
    slam_pcd.rotate(R, center=(0,0,0))
    slam_pcd.translate([TRANS_X, TRANS_Y, TRANS_Z])

    # 5. Visual Check
    print("\n👀 Opening Visualization... (Check if sides overlap!)")
    draw_registration_result(slam_pcd, gt_pcd, window_name="Check Alignment")

    # 6. Evaluation
    print("\n--- 📊 EVALUATION RESULTS ---")
    
    # Accuracy (SLAM -> GT)
    dists_s2g = slam_pcd.compute_point_cloud_distance(gt_pcd)
    accuracy = np.mean(dists_s2g)
    print(f"✅ Accuracy (Mean Error): {accuracy*100:.2f} cm")
    print("   (Target: < 2.0 cm)")

    # Completeness (GT -> SLAM)
    dists_g2s = gt_pcd.compute_point_cloud_distance(slam_pcd)
    completeness = np.mean(dists_g2s)
    print(f"⚠️ Completeness (Mean Error): {completeness*100:.2f} cm")
    print("   (Expected to be high due to missing top)")

if __name__ == "__main__":
    main()