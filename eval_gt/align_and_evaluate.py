import open3d as o3d
import numpy as np
import copy

# --- CONFIG ---
SLAM_FILE = "Object_0.ply"       # Your SLAM result (あなたのSLAM結果)
GT_FILE = "gt_cardboard_box.ply"      # Your scaled GT (in meters) (スケールされたGT)
VOXEL_SIZE = 0.05                       # For rough alignment (5cm)

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])    # SLAM = Yellow (SLAM = 黄色)
    target_temp.paint_uniform_color([0, 0.651, 0.929]) # GT = Blue (GT = 青色)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      window_name="Alignment Check")

def align_clouds(slam, gt):
    print("1. Centering Clouds...")
    # Move both to (0,0,0) to fix the "completely different position" issue
    # 「完全に異なる位置」の問題を修正するため、両方を (0,0,0) に移動
    slam_center = slam.get_center()
    gt_center = gt.get_center()
    slam.translate(-slam_center)
    gt.translate(-gt_center)

    print("2. Rough Alignment (Global)...")
    # This assumes they are roughly upright. If rotation is huge, we might need RANSAC.
    # （大まかに垂直であることを想定。回転が大きい場合はRANSACが必要になることも）
    threshold = 0.2 # 20cm distance threshold (20cmの距離しきい値)
    trans_init = np.identity(4)
    
    # Apply Point-to-Plane ICP (Best for walls/boxes)
    # Point-to-Plane ICPを適用（壁や箱に最適）
    # We need normals for Point-to-Plane (Point-to-Planeには法線が必要)
    slam.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    gt.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    reg_p2l = o3d.pipelines.registration.registration_icp(
        slam, gt, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    
    print(f"   Fitness: {reg_p2l.fitness:.4f} (Overlapping area)")
    print(f"   RMSE: {reg_p2l.inlier_rmse:.4f}")
    
    return reg_p2l.transformation

def evaluate_metrics(slam, gt):
    print("\n--- 📊 EVALUATION RESULTS ---")
    
    # 1. Accuracy (SLAM -> GT)
    # "How close are my points to the real wall?"
    # 「私の点群は実際の壁にどれだけ近いか？」
    dists_s2g = slam.compute_point_cloud_distance(gt)
    dists_s2g = np.asarray(dists_s2g)
    accuracy = np.mean(dists_s2g)
    print(f"✅ Accuracy (Mean Error): {accuracy*100:.2f} cm")
    
    # 2. Completeness (GT -> SLAM)
    # "How much of the box did I miss?"
    # 「箱のどれだけを見逃したか？」
    dists_g2s = gt.compute_point_cloud_distance(slam)
    dists_g2s = np.asarray(dists_g2s)
    completeness = np.mean(dists_g2s)
    print(f"⚠️ Completeness (Mean Error): {completeness*100:.2f} cm")
    print("   (Note: High completeness error is expected due to missing top/bottom)")
    print("   （注：上部/底部が欠落しているため、完全性誤差が高いのは想定内です）")

if __name__ == "__main__":
    # Load (ロード)
    print(f"Loading {SLAM_FILE} and {GT_FILE}...")
    slam_pcd = o3d.io.read_point_cloud(SLAM_FILE)
    gt_pcd = o3d.io.read_point_cloud(GT_FILE)

    # Align (位置合わせ)
    transformation = align_clouds(slam_pcd, gt_pcd)
    
    # Visual Check (Yellow = SLAM, Blue = GT) (視覚的な確認)
    print("Opening Visualization... (Close window to continue)")
    draw_registration_result(slam_pcd, gt_pcd, transformation)
    
    # Apply transformation permanently for evaluation (評価のために変換を永続的に適用)
    slam_pcd.transform(transformation)
    
    # Evaluate (評価)
    evaluate_metrics(slam_pcd, gt_pcd)