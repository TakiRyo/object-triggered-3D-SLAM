import open3d as o3d
import trimesh
import numpy as np
import copy
import os

# ==========================================
# 🛠️ CONFIGURATION
# ==========================================

# 1. FILE PATHS
PROPOSAL_FILE = "/home/ros2_env/taki/otslam/eval/eval_table_chair/table_chair_slam.ply"
RTAB_FILE     = "/home/ros2_env/taki/otslam/eval/eval_table_chair/rtab_table_chair.ply"

TABLE_FILE    = "/home/ros2_env/taki/otslam/eval/eval_table_chair/table_marble/meshes/table_lightmap.dae"
CHAIR_FILE    = "/home/ros2_env/taki/otslam/eval/eval_table_chair/Chair/meshes/Chair.obj"

# 2. GT SCALING & PLACEMENT (Based on your working script)
UNIT_SCALE = 1.0
TABLE_SCALE = [0.258, 0.258, 0.258] 
CHAIR_SCALE = [0.0075, 0.0075, 0.0075]

COMMON_Z = -0.6
TABLE_ROT   = [0.0, 0.0, -1.0];   TABLE_TRANS   = [0.035, 0.17, 0.0]
CHAIR_1_ROT = [0.0, 0.0, 0.0];    CHAIR_1_TRANS = [0.6, 1.45, COMMON_Z]
CHAIR_2_ROT = [0.0, 0.0, 0.0];    CHAIR_2_TRANS = [-0.45, 1.45, COMMON_Z]
CHAIR_3_ROT = [0.0, 0.0, 180.0];  CHAIR_3_TRANS = [-0.6, -1.25, COMMON_Z]
CHAIR_4_ROT = [0.0, 0.0, 180.0];  CHAIR_4_TRANS = [0.6, -1.25, COMMON_Z]

# ==========================================

def load_pcd(path, color=None):
    try:
        pcd = o3d.io.read_point_cloud(path)
        if pcd.is_empty(): return None
        # Center the cloud to align with the manually placed GT
        center = pcd.get_center()
        pcd.translate(-center)
        if color: pcd.paint_uniform_color(color)
        return pcd
    except: return None

def load_and_scale_gt(filename, scale_factors, color):
    try: mesh = trimesh.load(filename, force='mesh')
    except: return None
    if isinstance(mesh, trimesh.Scene): mesh = trimesh.util.concatenate(mesh.dump())
    
    # Sample points to turn mesh into point cloud for distance calc
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(mesh.sample(50000))
    
    # Scale
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
    pcd_copy.rotate(R, center=(0,0,0))
    pcd_copy.translate(trans)
    return pcd_copy

def calculate_metrics(source_pcd, target_gt, name):
    print(f"\n🔍 Evaluating {name}...")
    
    # 1. ACCURACY (Map -> GT) "Are the points correct?"
    dists_map_to_gt = source_pcd.compute_point_cloud_distance(target_gt)
    accuracy = np.mean(dists_map_to_gt) * 100 # Convert to cm
    
    # 2. COMPLETENESS (GT -> Map) "Did we capture everything?"
    dists_gt_to_map = target_gt.compute_point_cloud_distance(source_pcd)
    completeness = np.mean(dists_gt_to_map) * 100 # Convert to cm
    
    print(f"   ✅ Accuracy (Mean Error):     {accuracy:.2f} cm")
    print(f"   ⚠️ Completeness (Mean Error): {completeness:.2f} cm")
    return accuracy, completeness

def main():
    print("--- 📥 LOADING DATA ---")
    # Load Maps
    proposal = load_pcd(PROPOSAL_FILE, [1, 0.7, 0]) # Yellow
    rtab     = load_pcd(RTAB_FILE, [0, 0.6, 0.9])   # Blue
    
    if proposal is None or rtab is None:
        print("❌ Error: Could not load one of the PLY files.")
        return

    # Load & Assemble GT
    gt_table = load_and_scale_gt(TABLE_FILE, TABLE_SCALE, [1, 0, 0]) 
    gt_chair = load_and_scale_gt(CHAIR_FILE, CHAIR_SCALE, [1, 0, 0])
    
    gt_scene = apply_transform(gt_table, TABLE_ROT, TABLE_TRANS) + \
               apply_transform(gt_chair, CHAIR_1_ROT, CHAIR_1_TRANS) + \
               apply_transform(gt_chair, CHAIR_2_ROT, CHAIR_2_TRANS) + \
               apply_transform(gt_chair, CHAIR_3_ROT, CHAIR_3_TRANS) + \
               apply_transform(gt_chair, CHAIR_4_ROT, CHAIR_4_TRANS)

    # --- 📊 RUN EVALUATION ---
    print("\n========================================")
    print("       🏆 FINAL METRICS REPORT")
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
    # Shift them apart slightly so you can see them side-by-side
    proposal_vis = copy.deepcopy(proposal)
    proposal_vis.translate([-1.5, 0, 0])
    gt_vis_prop = copy.deepcopy(gt_scene)
    gt_vis_prop.translate([-1.5, 0, 0])
    
    rtab_vis = copy.deepcopy(rtab)
    rtab_vis.translate([1.5, 0, 0])
    gt_vis_rtab = copy.deepcopy(gt_scene)
    gt_vis_rtab.translate([1.5, 0, 0])

    o3d.visualization.draw_geometries([proposal_vis, gt_vis_prop, rtab_vis, gt_vis_rtab],
                                      window_name="Left: Proposal vs GT | Right: RTAB vs GT")

if __name__ == "__main__":
    main()