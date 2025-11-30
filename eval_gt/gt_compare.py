import open3d as o3d
import trimesh
import numpy as np
import os

# --- CONFIGURATION ---
# 1. File Paths
DAE_FILE = "/home/ros2_env/taki/otslam/eval_gt/cardboard_box/meshes/cardboard_box.dae"
GT_PCD_OUTPUT = "gt_cardboard_box.ply"
NUMBER_OF_POINTS = 100000 

# 2. Scaling (Fixing the Unit Mismatch)
# If your GT looks huge, it's likely in mm. Use 0.001 to convert to meters.
SCALE_FACTOR = 0.001 

# 3. Alignment (Matching Gazebo World Position)
# Look at /gazebo/model_states or your world file to find these values.
# POSITION: [x, y, z] (in meters)
GAZEBO_POSITION = [0.0, 0.0, 0.0]  

# ORIENTATION: [x, y, z, w] (Quaternion from Gazebo)
# If you only know Roll/Pitch/Yaw, set this to None and use the Euler section below.
GAZEBO_QUATERNION = [0.0, 0.0, 0.0, 1.0] 

# Helper to Create Transformation Matrix
def get_transform_matrix(pos, quat):
    """Creates a 4x4 transformation matrix from position and quaternion."""
    T = np.eye(4)
    
    # Translation
    T[:3, 3] = pos
    
    # Rotation (Quaternion to Matrix)
    # Gazebo gives (x, y, z, w), Open3D expects (w, x, y, z)
    # We reorder it here to be safe.
    x, y, z, w = quat
    R = o3d.geometry.get_rotation_matrix_from_quaternion([w, x, y, z])
    T[:3, :3] = R
    
    return T

def convert_dae_to_pcd():
    print(f"Processing: {DAE_FILE}")
    
    # --- STEP 1: LOAD MESH (Using Trimesh) ---
    try:
        mesh = trimesh.load(DAE_FILE, force='mesh')
    except Exception as e:
        print(f"Error loading DAE with trimesh: {e}")
        return

    if isinstance(mesh, trimesh.Scene):
        print("File is a Scene, concatenating geometry...")
        mesh = trimesh.util.concatenate(mesh.dump())

    # Export to temp PLY for Open3D
    temp_ply = "temp_conversion.ply"
    mesh.export(temp_ply)
    
    gt_mesh = o3d.io.read_triangle_mesh(temp_ply)
    if not gt_mesh.has_triangles():
        print("Error: Converted mesh has no triangles!")
        return

    print(f"Successfully loaded mesh. Vertices: {len(gt_mesh.vertices)}")

    # --- STEP 2: SAMPLE POINTS (Mesh -> Point Cloud) ---
    print(f"Sampling {NUMBER_OF_POINTS} points...")
    gt_pcd = gt_mesh.sample_points_uniformly(number_of_points=NUMBER_OF_POINTS)
    
    # --- STEP 3: SCALE (Millimeters -> Meters) ---
    # We scale around (0,0,0) because raw meshes are usually defined relative to origin.
    print(f"Scaling by factor: {SCALE_FACTOR}...")
    gt_pcd.scale(SCALE_FACTOR, center=(0, 0, 0))

    # --- STEP 4: ALIGNMENT (Place in Gazebo Coordinate Frame) ---
    print(f"Applying Transformation to position: {GAZEBO_POSITION}")
    transformation_matrix = get_transform_matrix(GAZEBO_POSITION, GAZEBO_QUATERNION)
    gt_pcd.transform(transformation_matrix)

    # --- STEP 5: SAVE ---
    o3d.io.write_point_cloud(GT_PCD_OUTPUT, gt_pcd)
    print(f"✅ Ground Truth Point Cloud saved to: {os.path.abspath(GT_PCD_OUTPUT)}")
    print(f"   Final Center: {gt_pcd.get_center()}")
    
    if os.path.exists(temp_ply):
        os.remove(temp_ply)

if __name__ == "__main__":
    convert_dae_to_pcd()