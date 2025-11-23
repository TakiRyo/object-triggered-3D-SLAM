import cv2, numpy as np
depth_path = "/home/ros2_env/taki/otslam/ros2_ws/src/rgbd_capture/object_scan/depth/depth_0001.png"
d = cv2.imread(depth_path, -1)
print("dtype:", d.dtype)
print("min:", np.min(d), "max:", np.max(d))
