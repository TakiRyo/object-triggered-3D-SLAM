"""
This script combines a 2D occupancy grid map (PGM format) with multiple 3D object reconstructions (PLY format)
to create a hybrid map. The user can manually adjust the position and orientation of each 3D object
relative to the 2D map using an interactive visualization tool.

Usage:
1. The 2D map is loaded from a PGM file and converted into a point cloud.
2. Each 3D object is loaded as a point cloud and displayed alongside the 2D map.
3. The user can adjust the position and orientation of the 3D object interactively.
4. The adjusted 3D object is merged with the 2D map.
5. The final hybrid map is saved as a PLY file.
"""

import yaml
import cv2
import numpy as np
import open3d as o3d
import os
import glob
import copy

# --- CONFIGURATION ---
# 2D map
map_base = "/home/ros2_env/taki/otslam/2d_map"
yaml_path = os.path.join(map_base, "map_selective.yaml")
pgm_path  = os.path.join(map_base, "map_selective.pgm")

# 3D map directory
obj_dir  = "/home/ros2_env/taki/otslam/3d_model/object_scan_update/3d_reconst"

# Output
save_path = "/home/ros2_env/taki/otslam/fusion/hybrid_maps/hybrid_map_selective_adjusted.ply"

# --- CONTROLS ---
TRANS_STEP = 0.05  # Translation step size (meters)
ROT_STEP   = 2.0   # Rotation step size (degrees)

class ManualAligner:
    def __init__(self, static_map_pcd, target_obj_pcd, obj_name):
        self.map_pcd = static_map_pcd
        self.obj_pcd = target_obj_pcd
        self.obj_name = obj_name
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        
        # Cumulative transformation matrix
        self.total_transformation = np.identity(4)

    def run(self):
        print(f"\n=== Adjusting Object: {self.obj_name} ===")
        print("  [W/S] Move X axis (Forward/Back)")
        print("  [A/D] Move Y axis (Left/Right)")
        print("  [Z/C] Rotate Yaw (Left/Right)")
        print("  [Q]   FINISH and Save this object")
        print("=========================================")

        self.vis.create_window(window_name=f"Aligning: {self.obj_name}", width=1280, height=720)
        
        # Add the map (static) and object (movable)
        self.vis.add_geometry(self.map_pcd)
        self.vis.add_geometry(self.obj_pcd)
        
        # Add coordinate axes (origin)
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        self.vis.add_geometry(axis)

        # --- Key Bindings ---
        # Translation
        self.vis.register_key_callback(ord("W"), self.move_x_pos)
        self.vis.register_key_callback(ord("S"), self.move_x_neg)
        self.vis.register_key_callback(ord("A"), self.move_y_pos)
        self.vis.register_key_callback(ord("D"), self.move_y_neg)
        
        # Rotation
        self.vis.register_key_callback(ord("Z"), self.rot_yaw_pos)
        self.vis.register_key_callback(ord("C"), self.rot_yaw_neg)
        
        # Finish
        self.vis.register_key_callback(ord("Q"), self.close_window)

        self.vis.run()
        self.vis.destroy_window()
        
        return self.obj_pcd

    # --- Transformation Functions ---
    def apply_trans(self, trans_matrix):
        self.obj_pcd.transform(trans_matrix)
        self.vis.update_geometry(self.obj_pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

    def move_x_pos(self, vis):
        T = np.eye(4); T[0, 3] = TRANS_STEP
        self.apply_trans(T)
        
    def move_x_neg(self, vis):
        T = np.eye(4); T[0, 3] = -TRANS_STEP
        self.apply_trans(T)

    def move_y_pos(self, vis):
        T = np.eye(4); T[1, 3] = TRANS_STEP
        self.apply_trans(T)

    def move_y_neg(self, vis):
        T = np.eye(4); T[1, 3] = -TRANS_STEP
        self.apply_trans(T)

    def rot_yaw_pos(self, vis):
        # Rotate around the center of the object
        center = self.obj_pcd.get_center()
        R = self.obj_pcd.get_rotation_matrix_from_xyz((0, 0, np.radians(ROT_STEP)))
        self.obj_pcd.rotate(R, center=center)
        self.vis.update_geometry(self.obj_pcd)
    
    def rot_yaw_neg(self, vis):
        center = self.obj_pcd.get_center()
        R = self.obj_pcd.get_rotation_matrix_from_xyz((0, 0, np.radians(-ROT_STEP)))
        self.obj_pcd.rotate(R, center=center)
        self.vis.update_geometry(self.obj_pcd)

    def close_window(self, vis):
        print(f"  -> Object {self.obj_name} Confirmed.")
        self.vis.close()


def create_map_cloud(yaml_file, pgm_file):
    print(f"Loading Map: {pgm_file}")
    if not os.path.exists(yaml_file): return None
    with open(yaml_file, 'r') as f: data = yaml.safe_load(f)
    res = data['resolution']
    origin = data['origin']
    ox, oy = origin[0], origin[1]
    
    img = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
    rows, cols = np.where(img < 100)
    
    points = []
    h = img.shape[0]
    for r, c in zip(rows, cols):
        wx = ox + (c * res)
        wy = oy + ((h - 1 - r) * res)
        points.append([wx, wy, 0.0])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.paint_uniform_color([0.3, 0.3, 0.3]) # Gray walls
    return pcd

def main():
    print("--- 1. Loading 2D Map ---")
    map_pcd = create_map_cloud(yaml_path, pgm_path)
    if map_pcd is None: return

    print("\n--- 2. Interactive Object Adjustment ---")
    ply_files = sorted(glob.glob(os.path.join(obj_dir, "*.ply")))
    
    final_merged_pcd = copy.deepcopy(map_pcd)
    
    if len(ply_files) == 0:
        print("No objects found.")
        return

    for f in ply_files:
        obj_name = os.path.basename(f)
        
        # Load object
        temp_pcd = o3d.io.read_point_cloud(f)
        
        # Convert Mesh to PCD if needed
        if len(temp_pcd.points) == 0:
            mesh = o3d.io.read_triangle_mesh(f)
            temp_pcd = mesh.sample_points_uniformly(number_of_points=15000)
        
        # Color Red
        temp_pcd.paint_uniform_color([1.0, 0.0, 0.0])

        # === LAUNCH ADJUSTER ===
        # Pass a COPY of the map so we don't mess it up, but we want to see it
        aligner = ManualAligner(map_pcd, temp_pcd, obj_name)
        adjusted_obj = aligner.run()
        
        # Merge
        final_merged_pcd += adjusted_obj

    print("\n--- 3. Saving Final Map ---")
    if not os.path.exists(os.path.dirname(save_path)):
        os.makedirs(os.path.dirname(save_path))
    
    o3d.io.write_point_cloud(save_path, final_merged_pcd)
    print(f"Saved to: {save_path}")
    
    # Final Preview
    o3d.visualization.draw_geometries([final_merged_pcd], window_name="Final Result")

if __name__ == "__main__":
    main()