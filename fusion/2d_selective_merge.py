import cv2
import numpy as np

# ============================
# パス設定
# ============================
# ベースとなる古い地図（全体があるやつ）
OLD_MAP_PATH = "/home/ros2_env/taki/otslam/2d_map/map_check_nov30.pgm"
# 更新情報を持った新しい地図（一部分だけのやつ）
# ※ 事前に位置合わせ済みのものを使うのがベストです
NEW_MAP_PATH = "/home/ros2_env/taki/otslam/2d_map/map_dec_11_copy.pgm"
# 保存先
OUTPUT_PATH = "/home/ros2_env/taki/otslam/2d_map/map_selective.pgm"
# ============================


def smart_paste(base_img, overlay_img, x, y, w, h):
    """
    overlay_img(新) の指定エリアを base_img(旧) に移植する
    ただし、overlay_img が Unknown(グレー) の場所は上書きしない
    """
    # 範囲チェック
    h_img, w_img = base_img.shape
    if x < 0 or y < 0 or x+w > w_img or y+h > h_img:
        return base_img

    roi_base = base_img[y:y+h, x:x+w]
    roi_new = overlay_img[y:y+h, x:x+w]

    # マスク作成: 新しい地図で情報がある場所（白か黒）
    unknown_pixel = 205
    threshold = 5
    has_data_mask = (roi_new < (unknown_pixel - threshold)) | (roi_new > (unknown_pixel + threshold))

    # 上書き実行
    roi_base[has_data_mask] = roi_new[has_data_mask]
    
    # 元画像に戻す
    base_img[y:y+h, x:x+w] = roi_base
    return base_img

def main():
    # 1. 読み込み
    print("画像を読み込んでいます...")
    old_img = cv2.imread(OLD_MAP_PATH, cv2.IMREAD_GRAYSCALE)
    new_img = cv2.imread(NEW_MAP_PATH, cv2.IMREAD_GRAYSCALE)

    if old_img is None or new_img is None:
        print("Error: 画像が見つかりません。パスを確認してください。")
        return

    # サイズ合わせ
    if old_img.shape != new_img.shape:
        print(f"Resizing New Map: {new_img.shape} -> {old_img.shape}")
        new_img = cv2.resize(new_img, (old_img.shape[1], old_img.shape[0]))

    # 作業用（結果）画像
    result_map = old_img.copy()

    # ウィンドウ設定
    win_src = "1. SOURCE: New Map (Select Here)"
    win_res = "2. RESULT: Updated Old Map"
    cv2.namedWindow(win_src, cv2.WINDOW_NORMAL)
    cv2.namedWindow(win_res, cv2.WINDOW_NORMAL)

    print("\n" + "="*50)
    print("【操作手順】")
    print("1. '1. SOURCE' ウィンドウで、反映させたいエリアをマウスで囲む")
    print("2. [SPACE] または [ENTER] を押して決定")
    print("   -> '2. RESULT' ウィンドウが更新されます")
    print("3. 何度でも繰り返せます")
    print("4. 終わったら [Esc] キーを押して保存・終了")
    print("="*50 + "\n")

    while True:
        # 結果を表示（最初は古い地図そのまま）
        cv2.imshow(win_res, result_map)
        cv2.waitKey(1) # 描画更新用

        # ROI選択 (新しい地図上で選択させる)
        # showCrosshair=Trueで十字線、fromCenter=Falseで左上からドラッグ
        try:
            r = cv2.selectROI(win_src, new_img, showCrosshair=True, fromCenter=False)
        except Exception as e:
            print("Selection Window Closed.")
            break
        
        # r = (x, y, w, h)
        x, y, w, h = int(r[0]), int(r[1]), int(r[2]), int(r[3])

        # 選択されなかった、またはキャンセルされた場合
        if w == 0 or h == 0:
            print("選択範囲なし。終了しますか？ (y/n)")
            k = cv2.waitKey(0)
            if k == 27 or k == ord('y'): # Esc or y
                break
            else:
                continue

        # 合成処理
        print(f"エリア更新中... (x={x}, y={y}, w={w}, h={h})")
        result_map = smart_paste(result_map, new_img, x, y, w, h)
        print("完了！次のエリアを選択してください。")

    # 保存
    cv2.imwrite(OUTPUT_PATH, result_map)
    cv2.destroyAllWindows()
    print(f"\n保存完了: {OUTPUT_PATH}")

if __name__ == "__main__":
    main()