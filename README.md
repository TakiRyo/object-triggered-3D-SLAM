# OTSLAM: Active Object Scanning System

## 📖 System Overview

This project implements an autonomous "Move & Scan" pipeline. It utilizes 2D LiDAR to detect and classify objects in the environment, tracks them over time, and coordinates a mobile robot to navigate around them. Upon reaching specific viewpoints, the robot stabilizes and captures synchronized RGB-D data and camera poses for 3D reconstruction.

### The Pipeline

1.  **Perception (`lidar_detection`)**: Raw LiDAR data is segmented into clusters. Walls are filtered out, and compact objects are tracked.
2.  **Mission Planning (`lidar_detection`)**: The system generates "Visiting Points" (Goals) around detected objects.
3.  **Orchestration (`system_manager`)**: A central manager commands the robot to drive to these points using **Nav2**.
4.  **Data Capture (`system_manager`)**: Once arrived, the robot freezes and captures high-quality dataset frames.

-----

## 📦 Package 1: Lidar Detection

*Focus: Raw Sensor Processing, Classification, and Tracking.*

### 1\. 🧩 LiDAR Cluster Classification

**Node Name:** `lidar_cluster_publisher`

Segments 2D LiDAR scans into distinct clusters and classifies them based on geometric properties (PCA linearity, size, density).

  * **Logic:**
    1.  **Clustering:** Groups points based on Euclidean distance (`gap_threshold`).
    2.  **PCA Analysis:** Calculates eigenvalues ($\lambda_0, \lambda_1$) to determine if a cluster is linear or blob-like.
    3.  **Classification:**
          * 🟩 **Wall:** Long, straight, dense.
          * 🟦 **Object:** Short, compact (obstacles/items).
          * 🟨 **Unknown:** Curved corners or irregular shapes.

| Topic / Parameter | Type | Description |
| :--- | :--- | :--- |
| **Sub** `/scan` | `LaserScan` | Raw LiDAR input. |
| **Pub** `/wall_clusters` | `PointCloud2` | Static environmental features. |
| **Pub** `/object_clusters` | `PointCloud2` | Candidates for tracking. |
| `wal_lin_max` | Param | Linearity threshold (Lower = stricter straight lines). |

### 2\. 🎯 Object Tracker & Goal Generator

**Node Name:** `object_cluster_marker`

Provides temporal persistence to objects and generates navigation goals. It filters out transient noise and creates a "Lock Zone" around valid objects.

  * **Features:**
      * **State Machine:** `Candidate` (Yellow) $\to$ `Stable` (Green) if tracked for `stability_time`.
      * **Goal Generation:** Creates 4 "Visiting Points" (North/South/East/West) around the object.
      * **Orientation Logic:** Calculates Yaw so the robot always **faces the object center** at the destination.
      * **Service Integration:** Can "Freeze" the map state (stop updating object positions) to prevent goal jitter during navigation.

| Topic / Service | Type | Description |
| :--- | :--- | :--- |
| **Sub** `/object_clusters` | `PointCloud2` | Input from classification node. |
| **Pub** `/object_visiting_points`| `MarkerArray` | Cyan arrows representing nav goals. |
| **Srv** `set_tracking_mode` | `SetBool` | `True`=Live Lidar, `False`=Frozen Memory. |

### 3\. 📡 Goal Sender (Mission Manager)

**Node Name:** `goal_sender`

Acts as the dispatch interface between the Tracker and the System Manager. It decides *which* point to visit next.

  * **Strategy (Greedy + Sticky):**
    1.  **Sticky Focus:** If currently visiting "Object A", it prioritizes other points belonging to "Object A" before switching to "Object B".
    2.  **Proximity:** If no current focus, it picks the physically closest target.
    3.  **Progress Tracking:** Marks points as "Visited" when the robot gets within `reach_threshold`.

| Topic | Description |
| :--- | :--- |
| **Pub** `/manager/target_pose` | The active target sent to the System Manager. |
| **Pub** `/goal_status` | Visual feedback: Red (Active), Green (Done), Grey (Queue). |

-----

## 📦 Package 2: System Manager

*Focus: Robot Control, State Machine, and Data Saving.*

### 1\. 🧠 System Manager

**Node Name:** `system_manager`

The central orchestrator connecting the Mission Manager, Nav2, and the Scanner. It implements a Finite State Machine: **IDLE $\to$ NAVIGATING $\to$ SCANNING**.

#### ★ Feature Spotlight: Smart Tracking Strategy

To handle sensor noise while moving, this node intelligently switches the Tracker's mode:

1.  **Searching (Unfreeze):** When a **NEW** Object ID is received, Lidar tracking is enabled to pinpoint the object's exact location.
2.  **Orbiting (Freeze):** When moving to a new viewpoint of the **SAME** object, tracking is **Frozen**. This prevents the center point from shifting due to occlusion as the robot moves around it.
3.  **Scanning (Freeze):** Upon arrival, tracking is forced Frozen to ensure coordinate stability during data capture.

| Action / Service | Role |
| :--- | :--- |
| `Maps_to_pose` | Client for Nav2 to move the robot. |
| `scan_object` | Triggers the ScannerNode upon arrival. |
| `set_tracking_mode` | Toggles the Tracker's Lidar update loop. |

### 2\. 📸 Scanner Node

**Node Name:** `scanner_node`

The "Photographer." Acts as an Action Server that saves a synchronized snapshot of the environment.

  * **The "Stop-and-Stare" Routine:**
    1.  **Buffer Flush:** Clears old images to ensure no motion blur from the navigation phase.
    2.  **Stabilization:** Waits for `wait_time` (e.g., 5.0s) to let robot vibrations settle.
    3.  **Capture:** Saves RGB, Depth, and TF Pose.
    4.  **Depth Conversion:** Converts raw float meters $\to$ 16-bit PNG (Millimeters) and patches `NaN` values.
    5.  **Cool Down:** Waits again before returning success to prevent immediate robot jerking.

**Output Directory Structure:**

```text
/output_dir
├── /color
│   └── label_N.jpg        # Standard RGB
├── /depth
│   └── label_N.png        # 16-bit Integer Depth (mm)
└── /poses
    └── label_N.txt        # 4x4 Matrix (World -> Camera)
```