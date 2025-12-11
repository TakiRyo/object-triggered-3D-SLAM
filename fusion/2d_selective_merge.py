"""
This script allows selective merging of two 2D occupancy grid maps using a graphical interface.
Users can select a rectangular region on the new map, and the corresponding area will be merged
into the old map. The merging process ensures that unknown areas in the new map do not overwrite
known areas in the old map.

Usage:
1. Drag a rectangle on the "SOURCE" window to select an area.
2. Press SPACE or ENTER to apply the selected area to the old map.
3. Press ESC to save the merged map and exit.
"""

import cv2
import numpy as np

# ============================
# Path Configuration
# ============================
OLD_MAP_PATH = "/home/ros2_env/taki/otslam/2d_map/map_check_nov30.pgm"
NEW_MAP_PATH = "/home/ros2_env/taki/otslam/2d_map/map_dec_11_copy.pgm" 
OUTPUT_PATH = "/home/ros2_env/taki/otslam/2d_map/map_selective.pgm"
# ============================

# Class to manage mouse operations
class DraggableRect:
    def __init__(self):
        self.start_point = None # Drag start point (x, y)
        self.end_point = None   # Current mouse position (x, y)
        self.dragging = False   # Flag to indicate if dragging is in progress
        self.selected_rect = None # Finalized rectangle (x, y, w, h)

    # Mouse event callback function
    def mouse_callback(self, event, x, y, flags, param):
        # Start dragging when the left button is pressed
        if event == cv2.EVENT_LBUTTONDOWN:
            self.start_point = (x, y)
            self.end_point = (x, y)
            self.dragging = True
            self.selected_rect = None # Start a new selection

        # Update the rectangle while dragging
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.end_point = (x, y)

        # Finalize the rectangle when the left button is released
        elif event == cv2.EVENT_LBUTTONUP:
            self.end_point = (x, y)
            self.dragging = False
            # Calculate the rectangle (top-left corner and width/height)
            x_min = min(self.start_point[0], self.end_point[0])
            y_min = min(self.start_point[1], self.end_point[1])
            w = abs(self.start_point[0] - self.end_point[0])
            h = abs(self.start_point[1] - self.end_point[1])
            if w > 0 and h > 0:
                self.selected_rect = (x_min, y_min, w, h)

# Merge function (same as before)
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
    # 1. Load images
    old_img = cv2.imread(OLD_MAP_PATH, cv2.IMREAD_GRAYSCALE)
    new_img = cv2.imread(NEW_MAP_PATH, cv2.IMREAD_GRAYSCALE)
    if old_img is None or new_img is None:
        print("Error: Images not found.")
        return
    if old_img.shape != new_img.shape:
        new_img = cv2.resize(new_img, (old_img.shape[1], old_img.shape[0]))

    # Convert to color for display (to draw blue lines)
    new_img_display = cv2.cvtColor(new_img, cv2.COLOR_GRAY2BGR)

    result_map = old_img.copy()
    
    win_src = "1. SOURCE: Select Area (Blue Box)"
    win_res = "2. RESULT: Updated Old Map"
    cv2.namedWindow(win_src, cv2.WINDOW_NORMAL)
    cv2.namedWindow(win_res, cv2.WINDOW_NORMAL)

    # Instantiate the mouse operation management class and set the callback
    rect_tool = DraggableRect()
    cv2.setMouseCallback(win_src, rect_tool.mouse_callback)

    print("\n" + "="*50)
    print("[Instructions]")
    print("1. In the '1. SOURCE' window, drag the mouse to select an area.")
    print("   -> **Blue rectangle** will appear.")
    print("2. Release the mouse to finish selecting.")
    print("3. Press **[SPACE] or [ENTER]** to apply the selection.")
    print("   -> The '2. RESULT' window will update.")
    print("4. Press [Esc] to save and exit.")
    print("="*50 + "\n")

    while True:
        # Copy the display image (reset and redraw each time)
        display_img = new_img_display.copy()

        # Draw a blue rectangle while dragging or after selection
        if rect_tool.start_point and rect_tool.end_point:
            # Color: Blue (BGR: (255, 0, 0)), Thickness: 2
            cv2.rectangle(display_img, rect_tool.start_point, rect_tool.end_point, (255, 0, 0), 2)

        # Display the images
        cv2.imshow(win_src, display_img)
        cv2.imshow(win_res, result_map)

        # Wait for key input
        key = cv2.waitKey(10) & 0xFF

        # Apply the selected area with [SPACE] or [ENTER]
        if key == 32 or key == 13: # Space or Enter
            if rect_tool.selected_rect:
                x, y, w, h = rect_tool.selected_rect
                print(f"Applying update to area: x={x}, y={y}, w={w}, h={h}")
                result_map = smart_paste(result_map, new_img, x, y, w, h)
                print("Done! Select the next area.")
                # Reset the selection state
                rect_tool.start_point = None
                rect_tool.end_point = None
                rect_tool.selected_rect = None
            else:
                print("No area selected. Drag with the mouse.")

        # Exit with [Esc]
        elif key == 27:
            break

    # Save the result
    cv2.imwrite(OUTPUT_PATH, result_map)
    cv2.destroyAllWindows()
    print(f"\nSaved: {OUTPUT_PATH}")

if __name__ == "__main__":
    main()