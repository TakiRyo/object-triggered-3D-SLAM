import cv2
import numpy as np
import yaml

def merge_maps(old_map_path, new_map_path, output_path):
    # 1. Load Images (Grayscale)
    # ROS Maps: 255 = Free (White), 0 = Occupied (Black), 205 = Unknown (Grey)
    old_img = cv2.imread(old_map_path, cv2.IMREAD_GRAYSCALE)
    new_img = cv2.imread(new_map_path, cv2.IMREAD_GRAYSCALE)

    if old_img is None or new_img is None:
        print("Error: Could not load images. Check paths.")
        return

    # Ensure sizes match (Resize old to match new if needed, or vice versa)
    # Ideally, they should be the same size if generated from the same start point.
    if old_img.shape != new_img.shape:
        print(f"Warning: Size mismatch! Old: {old_img.shape}, New: {new_img.shape}")
        print("Resizing Old Map to match New Map (This might distort if origins differ!)")
        old_img = cv2.resize(old_img, (new_img.shape[1], new_img.shape[0]))

    # 2. Define Masks
    # In ROS PGM:
    # 205 (approx) is Unknown.
    # We want to use NEW map pixels UNLESS they are 'Unknown'.
    
    # Create a mask where the NEW map has data (is NOT unknown)
    # We allow a small threshold around 205 just in case of compression noise
    unknown_pixel = 205
    threshold = 5 
    
    # Mask: True where New Map is KNOWN (Black or White)
    has_data_mask = (new_img < (unknown_pixel - threshold)) | (new_img > (unknown_pixel + threshold))

    # 3. Combine
    final_map = old_img.copy()
    
    # Wherever New Map has data, overwrite the Old Map
    final_map[has_data_mask] = new_img[has_data_mask]

    # 4. Save
    cv2.imwrite(output_path, final_map)
    print(f"Success! Merged map saved to: {output_path}")

# Define paths for the maps
old_whole_map = "/home/ros2_env/taki/otslam/2d_map/map_check_nov30.pgm"
new_part_map = "/home/ros2_env/taki/otslam/2d_map/map_dec_11_copy.pgm"
merge_map = "/home/ros2_env/taki/otslam/2d_map/map_merged_dec_11_2.pgm"

if __name__ == "__main__":
    # Use the defined paths
    merge_maps(old_whole_map, new_part_map, merge_map)