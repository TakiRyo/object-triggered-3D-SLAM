====================================
/**
 * ------------------------------------------------------------
 * LiDAR Cluster Classification Node
 * ------------------------------------------------------------
 * Purpose:
 * Segment 2D LiDAR scans into distinct clusters and classify 
 * them based on geometric properties (size, shape, density).
 *
 * Input:
 * /scan  (sensor_msgs::msg::LaserScan)
 *
 * Output:
 * /wall_clusters     (sensor_msgs::msg::PointCloud2) -> GREEN  [Long, straight structures]
 * /object_clusters   (sensor_msgs::msg::PointCloud2) -> BLUE   [Small, compact obstacles]
 * /unknown_clusters  (sensor_msgs::msg::PointCloud2) -> YELLOW [Everything else]
 *
 * Logic Flow:
 * 1. Clustering:
 * - Iterates through scan points.
 * - A new cluster begins if the Euclidean distance between consecutive 
 * points > `gap_threshold`.
 * - Merges the first and last cluster if the scan wraps around (360 degrees).
 *
 * 2. Feature Extraction (per cluster):
 * - Length: Diagonal distance of the axis-aligned bounding box.
 * - Linearity (PCA): Calculates covariance matrix and eigenvalues ($ \lambda_0, \lambda_1 $).
 * Score = $ \lambda_0 / \lambda_1 $. Near 0.0 implies a perfect line.
 *
 * 3. Classification Rules:
 * - WALL if:
 * Linearity < `wal_lin_max` AND 
 * Length    > `wal_len_min` AND 
 * Points    > `wal_nmp_min`
 * - OBJECT if:
 * Length    < `obj_len_max` AND 
 * Points    > `obj_nmp_min`
 * - Unknown:
 * Any cluster failing both above checks (e.g., curved corners, large irregular blobs).
 *
 * Adjustable Parameters:
 * - General:
 * max_range_ratio    : Ignore points beyond this % of sensor max range.
 * gap_threshold      : Distance (m) to break points into separate clusters.
 * min_cluster_points : Minimum points required to form a valid cluster.
 * wal_len_min        : Minimum length (m) to be considered a wall.
 * wal_lin_max        : Maximum PCA ratio (0.0=line, 1.0=blob). Lower is stricter.
 * wal_nmp_min        : Minimum point count for a wall.
 * obj_len_max        : Maximum length (m) to be considered an object.
 * obj_nmp_min        : Minimum point count for an object.
 * ------------------------------------------------------------
 */

 =====================================

 /**
 * -----------------------------------------------------------------------
 * Node Name: ObjectClusterMarker
 * -----------------------------------------------------------------------
 * Purpose:
 * 1. Tracks distinct objects over time (Temporal Persistence).
 * 2. Filters noise by requiring objects to be stable for `stability_time`.
 * 3. Generates "Visiting Points" (Goals) around the object.
 * 4. Freezes the map state via service for navigation tasks.
 *
 * Input:
 * /object_clusters (sensor_msgs::msg::PointCloud2)
 * - A single cloud containing points classified as "Objects" by the previous node.
 *
 * Output:
 * /candidate_clusters   (MarkerArray) -> YELLOW Boxes (Unstable/New detections)
 * /stable_clusters      (MarkerArray) -> GREEN Boxes  (Confirmed objects)
 * /debug_lock_zones     (MarkerArray) -> RED Cylinders (The "Keep-Out" or "Lock" zone)
 * /object_visiting_points (MarkerArray) -> CYAN Arrows (Navigation Goals)
 *
 * Logic Flow:
 * 1. Re-Clustering:
 * Incoming points are grouped into individual clusters using Euclidean distance.
 * 2. Data Association:
 * - New clusters are matched to existing 'Stable' objects first.
 * - If no match, they are matched to 'Candidate' objects.
 * - If still no match, a new 'Candidate' is created.
 * 3. State Machine:
 * - Candidate -> Stable: If tracked consistently for > `stability_time`.
 * - Candidate -> Deleted: If not seen for > 0.5 seconds.
 * 4. Goal Generation (The "Visiting Points"):
 * - Creates 4 points (North, South, East, West) around the object's lock zone.
 * - **Orientation Calculation**:
 * The orientation (Yaw) is calculated so the arrow points **FROM** the visiting point
 * **TO** the center of the object.
 * Formula: yaw = atan2(object_cy - point_y, object_cx - point_x)
 *
 * Services:
 * /set_tracking_mode (std_srvs::SetBool)
 * - True: Updates positions based on LiDAR (Live Tracking).
 * - False: Freezes markers in place (allows robot to navigate to them without
 * the goal jumping around due to sensor noise).
 *
 * Adjustable Parameters:
 * stability_time        : Seconds a cluster must exist to become "Stable".
 * lock_margin           : Extra padding added to object radius for the lock zone.
 * visiting_point_buffer : Distance from the lock zone to the goal points.
 * smoothing_factor      : 0.0 to 1.0. Higher = faster updates, Lower = smoother motion.
 * -----------------------------------------------------------------------
 */

 ==================

 /**
 * -----------------------------------------------------------------------
 * Node Name: GoalSender
 * -----------------------------------------------------------------------
 * Purpose:
 * Acts as the simple "Mission Manager" for the robot.
 * 1. Listens to potential visiting points (arrows) generated by the Tracker.
 * 2. Selects the *next best point* to visit based on proximity and object focus.
 * 3. Publishes the selected point as a navigation goal.
 * 4. Tracks progress: Marks points as "Visited" when the robot gets close.
 *
 * Input:
 * /object_visiting_points (visualization_msgs::msg::MarkerArray)
 * - The candidates (cyan arrows) from the ObjectClusterMarker node.
 * - Contains Position (x,y) and Orientation (facing the object).
 *
 * /odom (nav_msgs::msg::Odometry)
 * - The robot's current position to calculate distances.
 *
 * Output:
 * /manager/target_pose (geometry_msgs::msg::PoseStamped)
 * - The single, active goal for the robot controller to follow.
 * - Includes the specific orientation so the robot faces the object.
 *
 * /goal_status (visualization_msgs::msg::MarkerArray)
 * - Visual feedback for Rviz:
 * RED   = Currently Active Goal
 * GREEN = Visited / Done
 * GREY  = Waiting in queue
 *
 * Logic Flow:
 * 1. Parse & Filter:
 * - Reads incoming markers.
 * - Ignores IDs that are already in the `visited_ids_` set.
 *
 * 2. Reach Detection:
 * - Calculates distance: dist(Robot, ActiveGoal).
 * - If dist < `reach_threshold`, the point is marked as Visited.
 * - The Active Target is reset to -1 (searching).
 *
 * 3. Goal Selection Strategy (Greedy + Sticky):
 * - If we are already working on a specific Object ID (e.g., Object 1),
 * prioritize remaining points for Object 1 (minimize switching back and forth).
 * - If no current object focus, simply pick the *closest* available point.
 *
 * 4. Orientation Pass-through:
 * - The node does NOT calculate orientation. It simply forwards the
 * quaternion (qx, qy, qz, qw) received from the input marker.
 * This ensures the robot arrives facing the object center.
 *
 * Adjustable Parameters:
 * reach_threshold : Distance (m) to consider a goal "reached" (e.g., 0.6m).
 * -----------------------------------------------------------------------
 */


===============================
above three node is in lidar_detection package. about lidar perception
================================
below two node is system mangaer package. one is control pipeline, one is about camera capture
=================================
/*
 *-----------------------------------------------------------------------
 * Node Name: SystemManager
 * -----------------------------------------------------------------------
 * Purpose:
 * The central orchestrator (State Machine) for the "Move & Scan" mission.
 * It connects the Mission Manager (GoalSender), the Pilot (Nav2), 
 * the Photographer (ScannerNode), and the Tracker (ObjectClusterMarker).
 *
 * Role:
 * 1. Receives a target pose (Position + Orientation + Object ID).
 * 2. Navigates the robot to that pose.
 * 3. Upon arrival, triggers the precise data capture sequence.
 * 4. Manages the "Lidar vs. Camera" mode switching.
 *
 * Inputs:
 * /manager/target_pose (geometry_msgs::PoseStamped)
 * - From GoalSender. Contains the goal coordinates and the Object ID (in z).
 *
 * Actions & Services:
 * - Client: navigate_to_pose (Nav2) -> Moves the robot.
 * - Client: scan_object (ScannerNode) -> Captures Data.
 * - Client: set_tracking_mode (Service) -> Freezes/Unfreezes the object tracker.
 *
 * -----------------------------------------------------------------------
 * ★ SMART TRACKING STRATEGY (The "Freeze" Logic) ★
 * -----------------------------------------------------------------------
 * The critical logic in `goal_callback` handles when to trust the Lidar 
 * and when to trust the Memory (Frozen state):
 *
 * 1. NEW Object (ID changed):
 * Action: UNFREEZE (Enable Lidar Tracking).
 * Reason: We are approaching a new target. We need live Lidar data 
 * to find exactly where it is and center our markers.
 *
 * 2. SAME Object (ID unchanged, moving to next view):
 * Action: KEEP FROZEN.
 * Reason: We have already locked onto this object. As we move around 
 * it, parts of the object might become occluded, causing the center 
 * to shift wildly if we used live data. Keeping it frozen ensures 
 * all 4 visiting points remain relative to the *original* detected center.
 *
 * 3. ARRIVAL (Nav Success):
 * Action: FORCE FREEZE.
 * Reason: Ensure the coordinates are absolutely static during the 
 * 5-second camera stabilization phase.
 * -----------------------------------------------------------------------
 */

==============
/**
 * -----------------------------------------------------------------------
 * Node Name: ScannerNode
 * -----------------------------------------------------------------------
 * Purpose:
 * Acts as the Data Capture Action Server for the OTSLAM system.
 * It coordinates the camera and robot state to save a synchronized snapshot
 * of the environment (RGB Image + Depth Image + Camera Pose).
 *
 * Action Interface:
 * otslam_interfaces::action::ScanObject
 * - Goal: Label (string) -> e.g., "box"
 * - Result: Success/Failure status
 *
 * Inputs:
 * /rgb_topic   (sensor_msgs::Image) : Color stream
 * /depth_topic (sensor_msgs::Image) : Aligned depth stream
 * /tf          (tf2)                : Robot localization data
 *
 * Outputs (FileSystem):
 * Saves data to `output_dir` in the following structure:
 * - /color/label_N.jpg   : Standard RGB image
 * - /depth/label_N.png   : 16-bit PNG (Depth in millimeters)
 * - /poses/label_N.txt   : 4x4 Transformation Matrix (World -> Camera)
 *
 * Execution Logic (The "Stop-and-Stare" Routine):
 * 1. Buffer Flush:
 * Immediately clears the `latest_rgb_` and `latest_depth_` variables.
 * This ensures we do not save "stale" images from when the robot was moving.
 *
 * 2. Stabilization Wait (Pre-Scan):
 * Sleeps for `wait_time` (default 5.0s). This allows the robot's physical
 * vibrations to settle and ensures the camera buffer fills with *stationary* frames.
 *
 * 3. Capture & Verify:
 * Checks if new images have arrived during the wait. If valid, locks the
 * mutex and clones the data.
 *
 * 4. Save & Convert:
 * - rgb: Saved as JPG.
 * - Depth: NaN values patched to 0, converted to Millimeters (uint16).
 * - Pose: TF lookup (Map -> Camera) saved as a standard 4x4 Matrix.
 *
 * 5. Cool Down (Post-Scan):
 * Sleeps for `wait_time` again before returning "Success".
 * This prevents the Client from commanding the robot to move immediately,
 * ensuring the write operation is safe and the robot doesn't jerk away.
 *
 * Parameters:
 * wait_time (5.0s) : Time to wait BEFORE capturing and AFTER capturing.
 * output_dir       : Base path for saving datasets.
 * -----------------------------------------------------------------------
 */