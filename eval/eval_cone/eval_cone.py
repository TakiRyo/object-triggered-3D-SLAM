import open3d as o3d
import trimesh
import numpy as np
import copy
import os

# ==========================================
# 🛠️ CONFIGURATION
# ==========================================

# 1. FILE PATHS
# Proposal (OTSLAM)
PROPOSAL_FILE = "/home/ros2_env/taki/otslam/eval/eval_cone/cone_slam.ply"
# Baseline (RTAB-Map)
RTAB_FILE     = "/home/ros2_env/taki/otslam/eval/eval_cone/rtab_table_cone.ply"

# GT Files
GT_BLUE_FILE  = "/home/ros2_env/taki/otslam/eval/eval_cone/cone_blue/meshes/cone.stl"
GT_RED_FILE   = "/home/ros2_env/taki/otslam/eval/eval_cone/cone_red/meshes/cone.stl"

# 2. GT SCALING
UNIT_SCALE = 0.01     
SCALE_X = 1.0; SCALE_Y = 1.0; SCALE_Z = 1.0

# 3. MANUAL ALIGNMENT
COMMON_Z = -0.3

# Blue Cone Position
BLUE_ROT   = [0.0, 0.0, 0.0]
BLUE_TRANS = [0.5, 0.5, COMMON_Z]

# Red Cone Position
RED_ROT    = [0.0, 0.0, 0.0]
RED_TRANS  = [-0.395, -0.36, COMMON_Z]

# ==========================================

def load_pcd(path, color=None):
    try:
        pcd = o3d.io.read_point_cloud(path)
        if pcd.is_empty(): return None
        center = pcd.get_center()
        pcd.translate(-center)
        if color: pcd.paint_uniform_color(color)
        return pcd
    except: return None

def load_and_scale_gt(filename, color):
    try: mesh = trimesh.load(filename, force='mesh')
    except: return None
    if isinstance(mesh, trimesh.Scene): mesh = trimesh.util.concatenate(mesh.dump())
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(mesh.sample(50000))
    
    points = np.asarray(pcd.points) * UNIT_SCALE
    points[:, 0] *= SCALE_X; points[:, 1] *= SCALE_Y; points[:, 2] *= SCALE_Z
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

def calculate_metrics(source_pcd, target_gt, name):
    print(f"\n🔍 Evaluating {name}...")
    
    # 1. ACCURACY (Map -> GT)
    dists_map_to_gt = source_pcd.compute_point_cloud_distance(target_gt)
    accuracy = np.mean(dists_map_to_gt) * 100 # Convert to cm
    
    # 2. COMPLETENESS (GT -> Map)
    dists_gt_to_map = target_gt.compute_point_cloud_distance(source_pcd)
    completeness = np.mean(dists_gt_to_map) * 100 # Convert to cm
    
    print(f"   ✅ Accuracy (Mean Error):     {accuracy:.2f} cm")
    print(f"   ⚠️ Completeness (Mean Error): {completeness:.2f} cm")
    return accuracy, completeness

def main():
    print("--- 📥 LOADING DATA ---")
    
    proposal = load_pcd(PROPOSAL_FILE, [1, 0.7, 0]) # Yellow
    rtab     = load_pcd(RTAB_FILE, [0, 0.6, 0.9])   # Blue
    
    if proposal is None: print(f"❌ Error loading {PROPOSAL_FILE}"); return
    if rtab is None:     print(f"❌ Error loading {RTAB_FILE}"); return

    gt_blue = load_and_scale_gt(GT_BLUE_FILE, [1, 0, 0]) # Red
    gt_red  = load_and_scale_gt(GT_RED_FILE,  [1, 0, 0]) 
    
    gt_scene = apply_transform(gt_blue, BLUE_ROT, BLUE_TRANS) + \
               apply_transform(gt_red,  RED_ROT,  RED_TRANS)

    # --- 📊 RUN EVALUATION ---
    print("\n========================================")
    print("       🏆 CONE METRICS REPORT")
    print("========================================")

    acc_prop, comp_prop = calculate_metrics(proposal, gt_scene, "PROPOSAL (OTSLAM)")
    acc_rtab, comp_rtab = calculate_metrics(rtab, gt_scene, "BASELINE (RTAB-Map)")

    print("\n--- 📝 SUMMARY TABLE ---")
    print(f"{'Metric':<15} | {'Proposal':<10} | {'Baseline':<10} | {'Result':<15}")
    print("-" * 55)
    print(f"{'Accuracy':<15} | {acc_prop:.2f} cm    | {acc_rtab:.2f} cm    | {'✅ Proposal' if acc_prop < acc_rtab else '❌ Baseline'}")
    print(f"{'Completeness':<15} | {comp_prop:.2f} cm    | {comp_rtab:.2f} cm    | {'✅ Proposal' if comp_prop < comp_rtab else '❌ Baseline'}")
    
    # --- VISUALIZATION ---
    print("\n👀 Visualizing (Red=GT, Yellow=Proposal, Blue=RTAB)...")
    print("   Moving comparisons far apart...")
    
    # Shift Left by 5 meters
    proposal_vis = copy.deepcopy(proposal)
    proposal_vis.translate([-2.0, 0, 0])
    gt_vis_prop = copy.deepcopy(gt_scene)
    gt_vis_prop.translate([-2.0, 0, 0])
    
    # Shift Right by 5 meters
    rtab_vis = copy.deepcopy(rtab)
    rtab_vis.translate([2.0, 0, 0])
    gt_vis_rtab = copy.deepcopy(gt_scene)
    gt_vis_rtab.translate([2.0, 0, 0])

    o3d.visualization.draw_geometries([proposal_vis, gt_vis_prop, rtab_vis, gt_vis_rtab],
                                      window_name="Left: Proposal vs GT | Right: RTAB vs GT",
                                      width=1200, height=600)

if __name__ == "__main__":
    main()