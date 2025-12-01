import open3d as o3d
import numpy as np
import copy

# --- CONFIG ---
SLAM_FILE = "Object_0.ply"              
GT_FILE = "gt_cardboard_box.ply"      

# ==========================================
# 🎛️ MANUAL ADJUSTMENT (完全手動設定)
# ==========================================
# 1. 自動微調整(ICP)を使うか？
#    True  = 手動値を初期値として、最後はコンピュータに任せる
#    False = コンピュータを信用せず、手動値そのままで評価する（★今回はFalse推奨）
USE_ICP = False 

# 2. 回転 (Rotation) - 前回の成功値を入力
ROT_X = 0.0   
ROT_Y = 0.0   
ROT_Z = 0.0   

# 3. 位置ズレ (Translation) - 中心からの微調整 (Unit: Meters)
#    黄色(SLAM)をどっちに動かしたいか？
#    X = 赤矢印方向, Y = 緑矢印方向, Z = 青矢印方向
TRANS_X = 0.0  
TRANS_Y = 0.0   
TRANS_Z = -0.1   
# ==========================================

def get_manual_rotation_matrix(rx, ry, rz):
    rx, ry, rz = np.radians(rx), np.radians(ry), np.radians(rz)
    R = o3d.geometry.get_rotation_matrix_from_xyz((rx, ry, rz))
    return R

def draw_registration_result(source, target, window_name="Result"):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])     # SLAM = Yellow
    target_temp.paint_uniform_color([0, 0.651, 0.929]) # GT = Blue
    
    # 座標軸を表示 (Red=X, Green=Y, Blue=Z)
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0,0,0])
    
    o3d.visualization.draw_geometries([source_temp, target_temp, axes],
                                      window_name=window_name)

def align_and_evaluate():
    print(f"Loading {SLAM_FILE} and {GT_FILE}...")
    slam = o3d.io.read_point_cloud(SLAM_FILE)
    gt = o3d.io.read_point_cloud(GT_FILE)

    # 1. Centering (両方を中心に持ってくる)
    print("1. Centering Clouds...")
    slam.translate(-slam.get_center())
    gt.translate(-gt.get_center())

    # 2. Apply MANUAL Transform (手動補正)
    print(f"2. Applying Manual Transform...")
    print(f"   Rot(deg): {ROT_X}, {ROT_Y}, {ROT_Z}")
    print(f"   Trans(m): {TRANS_X}, {TRANS_Y}, {TRANS_Z}")
    
    # 回転
    R = get_manual_rotation_matrix(ROT_X, ROT_Y, ROT_Z)
    slam.rotate(R, center=(0,0,0))
    # 移動
    slam.translate([TRANS_X, TRANS_Y, TRANS_Z])

    # --- VISUAL CHECK ---
    print("\n👀 Check Alignment... (Close window to see score)")
    draw_registration_result(slam, gt, window_name="Manual Alignment Check")

    # 3. ICP (Optional)
    if USE_ICP:
        print("3. Running ICP (Fine-tuning)...")
        threshold = 0.1 
        slam.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        gt.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        reg_p2l = o3d.pipelines.registration.registration_icp(
            slam, gt, threshold, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        
        print(f"   ICP Fitness: {reg_p2l.fitness:.4f}")
        slam.transform(reg_p2l.transformation)
        draw_registration_result(slam, gt, window_name="After ICP Result")
    else:
        print("3. Skipping ICP (Using Manual Alignment Only)")

    # 4. Evaluation
    print("\n--- 📊 EVALUATION RESULTS ---")
    dists = slam.compute_point_cloud_distance(gt)
    accuracy = np.mean(dists)
    print(f"✅ Accuracy (Mean Error): {accuracy*100:.2f} cm")

    dists_g2s = gt.compute_point_cloud_distance(slam)
    completeness = np.mean(dists_g2s)
    print(f"⚠️ Completeness (Mean Error): {completeness*100:.2f} cm")

if __name__ == "__main__":
    align_and_evaluate()