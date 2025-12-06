import open3d as o3d
import trimesh
import numpy as np
import copy
import os

# ==========================================
# 🛠️ CONFIGURATION
# ==========================================

# 1. FILE PATHS
PROPOSAL_FILE = "/home/ros2_env/taki/otslam/eval/eval_cardboard/multi_prop_cardboard.ply"
RTAB_FILE     = "/home/ros2_env/taki/otslam/eval/eval_cardboard/rtab_cardboard.ply"
GT_FILE       = "/home/ros2_env/taki/otslam/eval/eval_cardboard/cardboard_box/meshes/cardboard_box.dae"

GT_PLY_PATH   = "/home/ros2_env/taki/otslam/eval/eval_cardboard/gt_cardboard.ply"
RESULT_PATH   = "/home/ros2_env/taki/otslam/eval/eval_cardboard/result_cardboard.ply"

# 2. GT SCALING (Specific to Cardboard)
UNIT_SCALE = 0.001
# SCALE_X = 1.25932
SCALE_X= 1.4
SCALE_Y = 1.00745
SCALE_Z = 0.7

# 3. ★ SEPARATE ALIGNMENT CONFIGURATION ★
# Adjust these to fix the "shift" for each method separately.
# NOTE: The Maps are centered to (0,0,0). The GT is spawned at (0,0,0).
# Use these offsets to fix rotation/translation errors.

# --- FOR PROPOSAL (Yellow Points) ---
OFFSET_PROP_TRANS = [-0.01, 0.0, 0.0]   # [x, y, z] e.g., [0.0, 0.05, 0.0]
OFFSET_PROP_ROT   = [0.0, 0.0, 0.0]   # [roll, pitch, yaw]

# --- FOR RTAB-MAP (Blue Points) ---
OFFSET_RTAB_TRANS = [0.0, 0.0, 0.0]
OFFSET_RTAB_ROT   = [0.0, 0.0, 0.0]

# 4. AUTO-ALIGNMENT SWITCH (ICP)
# If True, computer tries to snap GT to points automatically
USE_ICP_PROP = False
USE_ICP_RTAB = False

# ==========================================

def load_pcd(path, color=None):
    try:
        pcd = o3d.io.read_point_cloud(path)
        if pcd.is_empty(): return None
        # ★ Center to (0,0,0) for single object evaluation
        center = pcd.get_center()
        pcd.translate(-center)
        if color: pcd.paint_uniform_color(color)
        return pcd
    except: return None

def load_and_scale_gt(filename, color):
    print(f"🔨 Loading GT: {os.path.basename(filename)}")
    try: mesh = trimesh.load(filename, force='mesh')
    except: return None
    if isinstance(mesh, trimesh.Scene): mesh = trimesh.util.concatenate(mesh.dump())
    
    # Clean way to sample points using Open3D/Trimesh bridge
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(mesh.sample(50000))
    
    # Apply Scaling
    points = np.asarray(pcd.points) * UNIT_SCALE
    points[:, 0] *= SCALE_X
    points[:, 1] *= SCALE_Y
    points[:, 2] *= SCALE_Z
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Center GT to match the centered maps
    pcd.translate(-pcd.get_center())
    
    pcd.paint_uniform_color(color)
    return pcd

def apply_transform(pcd, rot, trans):
    rx, ry, rz = np.radians(rot[0]), np.radians(rot[1]), np.radians(rot[2])
    R = o3d.geometry.get_rotation_matrix_from_xyz((rx, ry, rz))
    pcd_copy = copy.deepcopy(pcd)
    pcd_copy.rotate(R, center=(0,0,0))
    pcd_copy.translate(trans)
    return pcd_copy

def refine_icp(source, target):
    print("   🤖 Running ICP...")
    reg = o3d.pipelines.registration.registration_icp(
        source, target, 0.05, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )
    source.transform(reg.transformation)
    return source

def calculate_metrics(source_pcd, target_gt, name):
    print(f"\n🔍 Evaluating {name}...")
    
    dists_map_to_gt = source_pcd.compute_point_cloud_distance(target_gt)
    accuracy = np.mean(dists_map_to_gt) * 100 
    
    dists_gt_to_map = target_gt.compute_point_cloud_distance(source_pcd)
    completeness = np.mean(dists_gt_to_map) * 100 
    
    print(f"   ✅ Accuracy (Mean Error):     {accuracy:.2f} cm")
    print(f"   ⚠️ Completeness (Mean Error): {completeness:.2f} cm")
    return accuracy, completeness

def main():
    print("--- 📥 LOADING DATA ---")
    
    proposal = load_pcd(PROPOSAL_FILE, [1, 0.7, 0]) # Yellow
    rtab     = load_pcd(RTAB_FILE, [0, 0.6, 0.9])   # Blue
    
    if proposal is None: print(f"❌ Error loading {PROPOSAL_FILE}"); return
    if rtab is None:     print(f"❌ Error loading {RTAB_FILE}"); return

    # --- 1. BUILD BASE GT ---
    gt_base = load_and_scale_gt(GT_FILE, [1, 0, 0]) 
    if gt_base is None: print("❌ Error loading GT"); return

    # --- 2. CREATE SEPARATE GT INSTANCES ---
    
    # GT for Proposal (Red)
    gt_prop = copy.deepcopy(gt_base)
    gt_prop = apply_transform(gt_prop, OFFSET_PROP_ROT, OFFSET_PROP_TRANS)
    if USE_ICP_PROP: gt_prop = refine_icp(gt_prop, proposal)
    gt_prop.paint_uniform_color([1, 0, 0]) 

    # GT for RTAB (Green)
    gt_rtab = copy.deepcopy(gt_base)
    gt_rtab = apply_transform(gt_rtab, OFFSET_RTAB_ROT, OFFSET_RTAB_TRANS)
    if USE_ICP_RTAB: gt_rtab = refine_icp(gt_rtab, rtab)
    gt_rtab.paint_uniform_color([1, 0, 0]) 

    # --- 📊 RUN EVALUATION ---
    print("\n========================================")
    print("     🏆 CARDBOARD METRICS REPORT")
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
    print("   Left: Proposal (Yellow) vs GT (Red)")
    print("   Right: RTAB (Blue) vs GT (Green)")
    
    # Shift Left for display
    vis_prop_map = copy.deepcopy(proposal).translate([-0.5, 0, 0])
    vis_prop_gt  = copy.deepcopy(gt_prop).translate([-0.5, 0, 0])
    
    # Shift Right for display
    vis_rtab_map = copy.deepcopy(rtab).translate([0.5, 0, 0])
    vis_rtab_gt  = copy.deepcopy(gt_rtab).translate([0.5, 0, 0])

    o3d.visualization.draw_geometries([vis_prop_map, vis_prop_gt, vis_rtab_map, vis_rtab_gt],
                                      window_name="Cardboard: Proposal vs RTAB-Map")

    # Save results
    print(f"💾 Saving result visualization to {RESULT_PATH}...")
    combined_vis = vis_prop_map + vis_prop_gt + vis_rtab_map + vis_rtab_gt
    o3d.io.write_point_cloud(RESULT_PATH, combined_vis)
    print("✅ Saved!")

if __name__ == "__main__":
    main()