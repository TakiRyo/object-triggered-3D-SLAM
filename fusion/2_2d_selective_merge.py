import cv2
import numpy as np

# ============================
# パス設定
# ============================
OLD_MAP_PATH = "/home/ros2_env/taki/otslam/2d_map/map_check_nov30.pgm"
NEW_MAP_PATH = "/home/ros2_env/taki/otslam/2d_map/map_dec_11_copy.pgm" 
OUTPUT_PATH = "/home/ros2_env/taki/otslam/2d_map/map_selective.pgm"
# ============================

# マウス操作の状態を管理するクラス
class DraggableRect:
    def __init__(self):
        self.start_point = None # ドラッグ開始点 (x, y)
        self.end_point = None   # 現在のマウス位置 (x, y)
        self.dragging = False   # ドラッグ中かどうかのフラグ
        self.selected_rect = None # 最終確定した矩形 (x, y, w, h)

    # マウスイベントのコールバック関数
    def mouse_callback(self, event, x, y, flags, param):
        # 左ボタンが押されたらドラッグ開始
        if event == cv2.EVENT_LBUTTONDOWN:
            self.start_point = (x, y)
            self.end_point = (x, y)
            self.dragging = True
            self.selected_rect = None # 新しい選択を開始

        # ドラッグ中（マウス移動）
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.end_point = (x, y)

        # 左ボタンが離されたらドラッグ終了
        elif event == cv2.EVENT_LBUTTONUP:
            self.end_point = (x, y)
            self.dragging = False
            # 矩形を確定させる (左上の座標と幅・高さを計算)
            x_min = min(self.start_point[0], self.end_point[0])
            y_min = min(self.start_point[1], self.end_point[1])
            w = abs(self.start_point[0] - self.end_point[0])
            h = abs(self.start_point[1] - self.end_point[1])
            if w > 0 and h > 0:
                self.selected_rect = (x_min, y_min, w, h)

# 合成関数（前回と同じ）
def smart_paste(base_img, overlay_img, x, y, w, h):
    h_img, w_img = base_img.shape
    if x < 0 or y < 0 or x+w > w_img or y+h > h_img:
        return base_img
    roi_base = base_img[y:y+h, x:x+w]
    roi_new = overlay_img[y:y+h, x:x+w]
    unknown_pixel = 205
    threshold = 5
    has_data_mask = (roi_new < (unknown_pixel - threshold)) | (roi_new > (unknown_pixel + threshold))
    roi_base[has_data_mask] = roi_new[has_data_mask]
    base_img[y:y+h, x:x+w] = roi_base
    return base_img

def main():
    # 1. 読み込み
    old_img = cv2.imread(OLD_MAP_PATH, cv2.IMREAD_GRAYSCALE)
    new_img = cv2.imread(NEW_MAP_PATH, cv2.IMREAD_GRAYSCALE)
    if old_img is None or new_img is None:
        print("Error: 画像が見つかりません。")
        return
    if old_img.shape != new_img.shape:
        new_img = cv2.resize(new_img, (old_img.shape[1], old_img.shape[0]))

    # 表示用にカラー変換（青い線を描くため）
    new_img_display = cv2.cvtColor(new_img, cv2.COLOR_GRAY2BGR)

    result_map = old_img.copy()
    
    win_src = "1. SOURCE: Select Area (Blue Box)"
    win_res = "2. RESULT: Updated Old Map"
    cv2.namedWindow(win_src, cv2.WINDOW_NORMAL)
    cv2.namedWindow(win_res, cv2.WINDOW_NORMAL)

    # マウス操作管理クラスのインスタンス化とコールバック設定
    rect_tool = DraggableRect()
    cv2.setMouseCallback(win_src, rect_tool.mouse_callback)

    print("\n" + "="*50)
    print("【操作手順 (改良版)】")
    print("1. '1. SOURCE' ウィンドウで、マウスをドラッグしてエリアを囲む")
    print("   -> **青い枠線** が表示されます")
    print("2. 囲み終わったらマウスを離す")
    print("3. **[SPACE] または [ENTER] キーを押して決定**")
    print("   -> '2. RESULT' ウィンドウが更新されます")
    print("4. 終わったら [Esc] キーで保存・終了")
    print("="*50 + "\n")

    while True:
        # 表示用の画像をコピー（毎回リセットして描画するため）
        display_img = new_img_display.copy()

        # ドラッグ中または選択完了後に、青い矩形を描画
        if rect_tool.start_point and rect_tool.end_point:
            # 色: 青 (BGRなので (255, 0, 0)), 線の太さ: 2
            cv2.rectangle(display_img, rect_tool.start_point, rect_tool.end_point, (255, 0, 0), 2)

        # 画像を表示
        cv2.imshow(win_src, display_img)
        cv2.imshow(win_res, result_map)

        # キー入力待ち
        key = cv2.waitKey(10) & 0xFF

        # [SPACE] か [ENTER] で選択範囲を適用
        if key == 32 or key == 13: # Space or Enter
            if rect_tool.selected_rect:
                x, y, w, h = rect_tool.selected_rect
                print(f"エリア更新実行: x={x}, y={y}, w={w}, h={h}")
                result_map = smart_paste(result_map, new_img, x, y, w, h)
                print("完了！次のエリアを選択してください。")
                # 選択状態をリセット
                rect_tool.start_point = None
                rect_tool.end_point = None
                rect_tool.selected_rect = None
            else:
                print("範囲が選択されていません。マウスでドラッグしてください。")

        # [Esc] で終了
        elif key == 27:
            break

    # 保存
    cv2.imwrite(OUTPUT_PATH, result_map)
    cv2.destroyAllWindows()
    print(f"\n保存完了: {OUTPUT_PATH}")

if __name__ == "__main__":
    main()