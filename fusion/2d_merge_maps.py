import cv2
import numpy as np

# ==========================================
# ★ ここでズレを調整してください (単位: ピクセル / 度)
# ==========================================
SHIFT_X = 0      # 右にズラす (+), 左にズラす (-)
SHIFT_Y = 0      # 下にズラす (+), 上にズラす (-)
ROTATION = 0.0   # 回転角度 (反時計回り +, 時計回り -)
# ==========================================

def merge_maps_manual(old_map_path, new_map_path, output_path):
    # 1. Load Images
    old_img = cv2.imread(old_map_path, cv2.IMREAD_GRAYSCALE)
    new_img = cv2.imread(new_map_path, cv2.IMREAD_GRAYSCALE)

    if old_img is None or new_img is None:
        print("Error: 画像が見つかりません")
        return

    # Resize (Base is old map)
    if old_img.shape != new_img.shape:
        print(f"Resizing New Map to match Old Map: {old_img.shape}")
        new_img = cv2.resize(new_img, (old_img.shape[1], old_img.shape[0]))

    height, width = old_img.shape

    # --- ★ 手動変換行列の作成 ---
    # 1. 回転行列を作成 (画像の中心を軸に回転)
    center = (width // 2, height // 2)
    M = cv2.getRotationMatrix2D(center, ROTATION, 1.0)

    # 2. 平行移動を加える
    M[0, 2] += SHIFT_X
    M[1, 2] += SHIFT_Y

    # 3. 変換適用 (背景はグレー205で埋める)
    new_img_aligned = cv2.warpAffine(
        new_img, 
        M, 
        (width, height), 
        flags=cv2.INTER_LINEAR, 
        borderValue=205
    )
    # -----------------------------

    # 2. Define Masks
    unknown_pixel = 205
    threshold = 5 
    
    # マスク: 位置合わせ後の新しいマップで「データがある場所」
    has_data_mask = (new_img_aligned < (unknown_pixel - threshold)) | (new_img_aligned > (unknown_pixel + threshold))

    # 3. Combine
    final_map = old_img.copy()
    
    # 4. Overwrite
    final_map[has_data_mask] = new_img_aligned[has_data_mask]

    # 5. Save
    cv2.imwrite(output_path, final_map)
    print(f"完了! Saved to: {output_path}")
    print(f"設定値 -> X: {SHIFT_X}, Y: {SHIFT_Y}, Rot: {ROTATION}")

# Define paths
old_whole_map = "/home/ros2_env/taki/otslam/2d_map/map_check_nov30.pgm"
new_part_map = "/home/ros2_env/taki/otslam/2d_map/map_dec_11_copy.pgm"
merge_map = "/home/ros2_env/taki/otslam/2d_map/map_merged_manual.pgm"

if __name__ == "__main__":
    merge_maps_manual(old_whole_map, new_part_map, merge_map)