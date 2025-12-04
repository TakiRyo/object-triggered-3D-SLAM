import open3d as o3d
import trimesh
import numpy as np
import copy
import os

# ==========================================
# 🛠️ CONFIGURATION
# ==========================================

# 1. FILE PATHS
PROPOSAL_FILE = "/home/ros2_env/taki/otslam/eval/eval_cardboard/prop_cardboard.ply"
RTAB_FILE     = "/home/ros2_env/taki/otslam/eval/eval_cardboard/rtab_cardboard.ply"
GT_FILE       = "/home/ros2_env/taki/otslam/eval/eval_cardboard/cardboard_box/meshes/cardboard_box.dae"
GT_PLY_PATH   = "/home/ros2_env/taki/otslam/eval/eval_cardboard/gt_cardboard.ply"
RESULT_PATH ="/home/ros2_env/taki/otslam/eval/eval_cardboard/result_cardboard.ply"

# 2. GT SCALING (Specific to this object)
# <scale>1.25932 1.00745 0.6</scale>
UNIT_SCALE = 0.001
SCALE_X = 1.25932
SCALE_Y = 1.00745
SCALE_Z = 0.6

# 3. MANUAL ALIGNMENT
# Shifts the SLAM clouds to match the centered GT
# Based on your finding: Z needs to go down 2cm
ALIGN_TRANS = [0.0, 0.0, -0.02] 
ALIGN_ROT   = [0.0, 0.0, 0.0]

# ==========================================

def load_pcd(path, color=None):
    try:
        pcd = o3d.io.read_point_cloud(path)
        if pcd.is_empty(): return None
        # Center to (0,0,0) so it matches the GT's origin
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
    
    # Export/Import to sample points
    mesh.export("temp_gt_box.ply")
    gt_mesh = o3d.io.read_triangle_mesh("temp_gt_box.ply")
    if os.path.exists("temp_gt_box.ply"): os.remove("temp_gt_box.ply")
    
    # Sample points
    pcd = gt_mesh.sample_points_uniformly(number_of_points=10000)
    
    # Apply Non-Uniform Scale
    points = np.asarray(pcd.points) * UNIT_SCALE
    points[:, 0] *= SCALE_X
    points[:, 1] *= SCALE_Y
    points[:, 2] *= SCALE_Z
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Center GT
    pcd.translate(-pcd.get_center())
    
    pcd.paint_uniform_color(color)
    return pcd

def apply_alignment(pcd, rot, trans):
    """Applies the manual offset to the SLAM cloud"""
    rx, ry, rz = np.radians(rot[0]), np.radians(rot[1]), np.radians(rot[2])
    R = o3d.geometry.get_rotation_matrix_from_xyz((rx, ry, rz))
    pcd.rotate(R, center=(0,0,0))
    pcd.translate(trans)
    return pcd

def calculate_metrics(source_pcd, target_gt, name):
    print(f"\n🔍 Evaluating {name}...")
    
    # 1. ACCURACY (Map -> GT)
    dists_map_to_gt = source_pcd.compute_point_cloud_distance(target_gt)
    accuracy = np.mean(dists_map_to_gt) * 100 # cm
    
    # 2. COMPLETENESS (GT -> Map)
    dists_gt_to_map = target_gt.compute_point_cloud_distance(source_pcd)
    completeness = np.mean(dists_gt_to_map) * 100 # cm
    
    print(f"   ✅ Accuracy (Mean Error):     {accuracy:.2f} cm")
    print(f"   ⚠️ Completeness (Mean Error): {completeness:.2f} cm")
    return accuracy, completeness

def main():
    print("--- 📥 LOADING DATA ---")
    
    # 1. Load Maps
    proposal = load_pcd(PROPOSAL_FILE, [1, 0.7, 0]) # Yellow
    rtab     = load_pcd(RTAB_FILE, [0, 0.6, 0.9])   # Blue
    
    if proposal is None: print(f"❌ Error loading {PROPOSAL_FILE}"); return
    if rtab is None:     print(f"❌ Error loading {RTAB_FILE}"); return

    # 2. Load GT
    gt_pcd = load_and_scale_gt(GT_FILE, [1, 0, 0]) # Red
    if gt_pcd is None: print("❌ Error loading GT"); return

    # 3. Apply Manual Alignment (Shift SLAM down 2cm)
    print(f"🔄 Aligning SLAM clouds (Z shift: {ALIGN_TRANS[2]}m)...")
    proposal = apply_alignment(proposal, ALIGN_ROT, ALIGN_TRANS)
    rtab     = apply_alignment(rtab, ALIGN_ROT, ALIGN_TRANS)

    # --- 📊 RUN EVALUATION ---
    print("\n========================================")
    print("     🏆 CARDBOARD METRICS REPORT")
    print("========================================")

    acc_prop, comp_prop = calculate_metrics(proposal, gt_pcd, "PROPOSAL (OTSLAM)")
    acc_rtab, comp_rtab = calculate_metrics(rtab, gt_pcd, "BASELINE (RTAB-Map)")

    print("\n--- 📝 SUMMARY TABLE ---")
    print(f"{'Metric':<15} | {'Proposal':<10} | {'Baseline':<10} | {'Result':<15}")
    print("-" * 55)
    print(f"{'Accuracy':<15} | {acc_prop:.2f} cm    | {acc_rtab:.2f} cm    | {'✅ Proposal' if acc_prop < acc_rtab else '❌ Baseline'}")
    print(f"{'Completeness':<15} | {comp_prop:.2f} cm    | {comp_rtab:.2f} cm    | {'✅ Proposal' if comp_prop < comp_rtab else '❌ Baseline'}")
    
    # --- VISUALIZATION ---
    print("\n👀 Visualizing (Red=GT, Yellow=Proposal, Blue=RTAB)...")
    
    # Shift Left (Proposal vs GT)
    proposal_vis = copy.deepcopy(proposal)
    proposal_vis.translate([-0.5, 0, 0])
    gt_vis_prop = copy.deepcopy(gt_pcd)
    gt_vis_prop.translate([-0.5, 0, 0])
    
    # Shift Right (RTAB vs GT)
    rtab_vis = copy.deepcopy(rtab)
    rtab_vis.translate([0.5, 0, 0])
    gt_vis_rtab = copy.deepcopy(gt_pcd)
    gt_vis_rtab.translate([0.5, 0, 0])

    o3d.visualization.draw_geometries([proposal_vis, gt_vis_prop, rtab_vis, gt_vis_rtab],
                                      window_name="Cardboard: Proposal vs RTAB-Map",
                                      width=1200, height=600)

    # Save the scaled and aligned GT as a PLY file
    output_gt_path = GT_PLY_PATH
    print(f"💾 Saving GT to {output_gt_path}...")
    o3d.io.write_point_cloud(output_gt_path, gt_pcd)
    print("✅ GT saved successfully!")

    # Save the combined visualization as a PLY file
    
    output_vis_path = RESULT_PATH
    print(f"💾 Saving visualization to {output_vis_path}...")
    combined_vis = proposal_vis + gt_vis_prop + rtab_vis + gt_vis_rtab
    o3d.io.write_point_cloud(output_vis_path, combined_vis)
    print("✅ Visualization saved successfully!")

if __name__ == "__main__":
    main()