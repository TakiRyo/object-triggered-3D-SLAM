Nov. 7  
i could attach depth camera to turtlebot3 waffle. so this robot has RGBD camera.  

Nov. 23
i could create 3d local model. next step is to make whole pipeline. 

Nov 27
 * Node Name: ScannerNode
 * Role: Data Capture Action Server for 3D Reconstruction
 * * Functionality:
 * 1. Acts as a "Smart Camera Shutter" triggered via Action Goal.
 * 2. Pauses for 2 seconds to stabilize the robot (prevents motion blur).
 * 3. Captures synchronized RGB and Depth images from the camera.
 * 4. Retrieves the exact Camera-to-Map pose using TF2.
 * 5. Saves dataset files with incremental naming (e.g., Label_1, Label_2):
 * - Color: .jpg
 * - Depth: .png (Converted to 16-bit unsigned int for 3D tools)
 * - Pose:  .txt (4x4 Transformation Matrix)

 * Node Name: SystemManager
 * Role: Central State Machine for "Move & Scan" Behavior
 * Functionality:
 * 1. Listens for a target pose (from GoalSender).
 * 2. Extracts the Target ID from the pose.position.z field.
 * 3. STATE 1 (NAVIGATING): Sends goal to Nav2 to drive to the object.
 * 4. STATE 2 (SCANNING): Upon arrival, triggers the ScannerNode to save data.
 * 5. STATE 3 (IDLE): Resets and waits for the next target.
 * * Prevents overlapping commands by ignoring inputs while busy.

