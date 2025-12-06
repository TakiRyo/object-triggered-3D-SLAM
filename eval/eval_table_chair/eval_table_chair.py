#3d model

import open3d as o3d
import trimesh
import numpy as np
import copy
import os

# ==========================================
# 🛠️ CONFIGURATION
# ==========================================

# 1. FILE PATHS
PROPOSAL_FILE = "/home/ros2_env/taki/otslam/eval/eval_table_chair/multi_prop_table_chair.ply"
RTAB_FILE     = "/home/ros2_env/taki/otslam/eval/eval_table_chair/rtab_table_chair.ply"

TABLE_FILE    = "/home/ros2_env/taki/otslam/eval/eval_table_chair/table_marble/meshes/table_lightmap.dae"
CHAIR_FILE    = "/home/ros2_env/taki/otslam/eval/eval_table_chair/Chair/meshes/Chair.obj"

GT_PLY_PATH   = "/home/ros2_env/taki/otslam/eval/eval_table_chair/gt_table_chair.ply"
RESULT_PATH   = "/home/ros2_env/taki/otslam/eval/eval_table_chair/result_table_chair.ply"

# 2. GT CONSTRUCTION (Base Scaling & Placement)
UNIT_SCALE = 1.0
TABLE_SCALE = [0.258, 0.258, 0.258] 
CHAIR_SCALE = [0.0075, 0.0075, 0.0075]

COMMON_Z = -0.6
TABLE_ROT   = [0.0, 0.0, -1.0];   TABLE_TRANS   = [0.035, 0.17, 0.0]
CHAIR_1_ROT = [0.0, 0.0, 0.0];    CHAIR_1_TRANS = [0.6, 1.45, COMMON_Z]
CHAIR_2_ROT = [0.0, 0.0, 0.0];    CHAIR_2_TRANS = [-0.45, 1.45, COMMON_Z]
CHAIR_3_ROT = [0.0, 0.0, 180.0];  CHAIR_3_TRANS = [-0.6, -1.25, COMMON_Z]
CHAIR_4_ROT = [0.0, 0.0, 180.0];  CHAIR_4_TRANS = [0.6, -1.25, COMMON_Z]

# 3. ★ SEPARATE ALIGNMENT CONFIGURATION ★
# Since Proposal and RTAB might have different global frames, 
# we shift the GT specifically for the Proposal here.

# Shift GT to match Proposal (Adjust these based on visual error!)
# If Proposal is to the LEFT of GT, move GT LEFT (-x)
GT_OFFSET_FOR_PROPOSAL_TRANS = [-2.3, -3.5, 0.5]  # [x, y, z] in meters
GT_OFFSET_FOR_PROPOSAL_ROT   = [0.0, 0.0, 0.0]  # [rx, ry, rz] in degrees

# (Optional) Shift GT to match RTAB (Usually 0 if base config is correct)
GT_OFFSET_FOR_RTAB_TRANS     = [-2.3, -3.5, 0.5]
GT_OFFSET_FOR_RTAB_ROT       = [0.0, 0.0, 0.0]

# 4. AUTO-ALIGNMENT (ICP) SWITCH
# Set True to automatically snap GT to the Map (removes human error)
USE_ICP_FOR_PROPOSAL = False 
USE_ICP_FOR_RTAB     = False

# ==========================================

def load_pcd(path, color=None):
    try:
        pcd = o3d.io.read_point_cloud(path)
        if pcd.is_empty(): return None
        # Note: Do NOT center pcd here if you want to preserve relative SLAM coords
        # center = pcd.get_center() 
        # pcd.translate(-center) 
        if color: pcd.paint_uniform_color(color)
        return pcd
    except: return None

def load_and_scale_gt(filename, scale_factors, color):
    try: mesh = trimesh.load(filename, force='mesh')
    except: return None
    if isinstance(mesh, trimesh.Scene): mesh = trimesh.util.concatenate(mesh.dump())
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(mesh.sample(50000))
    
    points = np.asarray(pcd.points) * UNIT_SCALE
    points[:, 0] *= scale_factors[0]
    points[:, 1] *= scale_factors[1]
    points[:, 2] *= scale_factors[2]
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.paint_uniform_color(color)
    return pcd

def apply_transform(pcd, rot, trans):
    rx, ry, rz = np.radians(rot[0]), np.radians(rot[1]), np.radians(rot[2])
    R = o3d.geometry.get_rotation_matrix_from_xyz((rx, ry, rz))
    pcd_copy = copy.deepcopy(pcd)
    pcd_copy.rotate(R, center=(0,0,0)) # Rotate around origin
    pcd_copy.translate(trans)
    return pcd_copy

def refine_alignment_icp(source, target, threshold=0.05):
    """
    Automatically aligns source (GT) to target (Map) using ICP.
    threshold: Max distance (meters) to search for correspondences.
    """
    print("   🤖 Running ICP Fine-Tuning...")
    # Initial alignment is Identity because we assume manual is close
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )
    # Transform source to match target
    source.transform(reg_p2p.transformation)
    return source

def calculate_metrics(source_pcd, target_gt, name):
    print(f"\n🔍 Evaluating {name}...")
    
    # Accuracy: Map -> GT
    dists_map_to_gt = source_pcd.compute_point_cloud_distance(target_gt)
    accuracy = np.mean(dists_map_to_gt) * 100 
    
    # Completeness: GT -> Map
    dists_gt_to_map = target_gt.compute_point_cloud_distance(source_pcd)
    completeness = np.mean(dists_gt_to_map) * 100 
    
    print(f"   ✅ Accuracy (Mean Error):     {accuracy:.2f} cm")
    print(f"   ⚠️ Completeness (Mean Error): {completeness:.2f} cm")
    return accuracy, completeness

def main():
    print("--- 📥 LOADING DATA ---")
    proposal = load_pcd(PROPOSAL_FILE, [1, 0.7, 0]) # Yellow
    rtab     = load_pcd(RTAB_FILE, [0, 0.6, 0.9])   # Blue
    
    if proposal is None or rtab is None:
        print("❌ Error: Could not load PLY files.")
        return

    # --- ASSEMBLE BASE GT ---
    gt_table = load_and_scale_gt(TABLE_FILE, TABLE_SCALE, [1, 0, 0]) 
    gt_chair = load_and_scale_gt(CHAIR_FILE, CHAIR_SCALE, [1, 0, 0])
    
    gt_base = apply_transform(gt_table, TABLE_ROT, TABLE_TRANS) + \
              apply_transform(gt_chair, CHAIR_1_ROT, CHAIR_1_TRANS) + \
              apply_transform(gt_chair, CHAIR_2_ROT, CHAIR_2_TRANS) + \
              apply_transform(gt_chair, CHAIR_3_ROT, CHAIR_3_TRANS) + \
              apply_transform(gt_chair, CHAIR_4_ROT, CHAIR_4_TRANS)

    # --- 🔀 BRANCHING GT ---
    print("\n--- 🛠️ PREPARING GT VARIANTS ---")

    # 1. GT for PROPOSAL
    gt_prop = copy.deepcopy(gt_base)
    # Apply Manual Offset
    gt_prop = apply_transform(gt_prop, GT_OFFSET_FOR_PROPOSAL_ROT, GT_OFFSET_FOR_PROPOSAL_TRANS)
    # Apply ICP (Auto-fix) if enabled
    if USE_ICP_FOR_PROPOSAL:
        gt_prop = refine_alignment_icp(gt_prop, proposal)
    gt_prop.paint_uniform_color([1, 0, 0]) # Red for GT

    # 2. GT for RTAB
    gt_rtab = copy.deepcopy(gt_base)
    # Apply Manual Offset
    gt_rtab = apply_transform(gt_rtab, GT_OFFSET_FOR_RTAB_ROT, GT_OFFSET_FOR_RTAB_TRANS)
    # Apply ICP if enabled
    if USE_ICP_FOR_RTAB:
        gt_rtab = refine_alignment_icp(gt_rtab, rtab)
    gt_rtab.paint_uniform_color([1, 0, 0]) # Red for GT (RTAB) to distinguish
    

    # --- 📊 RUN EVALUATION ---
    print("\n========================================")
    print("       🏆 FINAL METRICS REPORT")
    print("========================================")

    acc_prop, comp_prop = calculate_metrics(proposal, gt_prop, "PROPOSAL (OTSLAM)")
    acc_rtab, comp_rtab = calculate_metrics(rtab, gt_rtab, "BASELINE (RTAB-Map)")

    print("\n--- 📝 SUMMARY TABLE ---")
    print(f"{'Metric':<15} | {'Proposal':<10} | {'Baseline':<10} | {'Result':<15}")
    print("-" * 55)
    print(f"{'Accuracy':<15} | {acc_prop:.2f} cm    | {acc_rtab:.2f} cm    | {'✅ Proposal' if acc_prop < acc_rtab else '❌ Baseline'}")
    print(f"{'Completeness':<15} | {comp_prop:.2f} cm    | {comp_rtab:.2f} cm    | {'✅ Proposal' if comp_prop < comp_rtab else '❌ Baseline'}")
    
    # --- VISUALIZATION ---
    print("\n👀 Visualizing...")
    
    # VISUALIZATION 1: PROPOSAL vs ITS GT
    # Move them to the LEFT side
    proposal_vis = copy.deepcopy(proposal).translate([-1.5, 0, 0])
    gt_prop_vis  = copy.deepcopy(gt_prop).translate([-1.5, 0, 0])
    
    # VISUALIZATION 2: RTAB vs ITS GT
    # Move them to the RIGHT side
    rtab_vis    = copy.deepcopy(rtab).translate([1.5, 0, 0])
    gt_rtab_vis = copy.deepcopy(gt_rtab).translate([1.5, 0, 0])

    o3d.visualization.draw_geometries([proposal_vis, gt_prop_vis, rtab_vis, gt_rtab_vis],
                                      window_name="Left: OTSLAM vs GT(Red) | Right: RTAB vs GT(Green)")

    # Save
    o3d.io.write_point_cloud(RESULT_PATH, proposal_vis + gt_prop_vis + rtab_vis + gt_rtab_vis)

if __name__ == "__main__":
    main()