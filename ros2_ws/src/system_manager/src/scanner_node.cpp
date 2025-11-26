#include <memory>
#include <thread>
#include <chrono>
#include <string>
#include <mutex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <map> // <--- Added for counter map

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "otslam_interfaces/action/scan_object.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp" 
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class ScannerNode : public rclcpp::Node
{
public:
  using ScanAction = otslam_interfaces::action::ScanObject;
  using GoalHandleScan = rclcpp_action::ServerGoalHandle<ScanAction>;

  ScannerNode() : Node("scanner_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    using namespace std::placeholders;

    // --- Parameters ---
    declare_parameter("rgb_topic", "/intel_realsense_r200_rgb/image_raw");
    declare_parameter("depth_topic", "/intel_realsense_r200_depth/depth/image_raw");
    declare_parameter("world_frame", "map");
    declare_parameter("camera_frame", "camera_rgb_optical_frame");
    declare_parameter("output_dir", "/home/ros2_env/taki/otslam/3d_model/object_scan_2");

    rgb_topic_ = get_parameter("rgb_topic").as_string();
    depth_topic_ = get_parameter("depth_topic").as_string();
    world_frame_ = get_parameter("world_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    output_dir_ = get_parameter("output_dir").as_string();

    // --- Directories ---
    std::filesystem::create_directories(output_dir_ + "/color");
    std::filesystem::create_directories(output_dir_ + "/depth");
    std::filesystem::create_directories(output_dir_ + "/poses");

    // --- Subscribers ---
    // Best Effort reliability is often better for sensor data
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
        rgb_topic_, qos, std::bind(&ScannerNode::rgb_callback, this, _1));
    
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, qos, std::bind(&ScannerNode::depth_callback, this, _1));

    // --- Action Server ---
    action_server_ = rclcpp_action::create_server<ScanAction>(
      this,
      "scan_object", 
      std::bind(&ScannerNode::handle_goal, this, _1, _2),
      std::bind(&ScannerNode::handle_cancel, this, _1),
      std::bind(&ScannerNode::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "=== Real Scanner Ready ===");
    RCLCPP_INFO(this->get_logger(), "Saving to: %s", output_dir_.c_str());
  }

private:
  // Map to store counters for each object (e.g., "Object_0" -> 1, "Object_1" -> 3)
  std::map<std::string, int> object_counters_; 

  // --- Action Handlers ---
  rclcpp_action::Server<ScanAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ScanAction::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Request: Capture '%s'", goal->label.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleScan> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Request canceled.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleScan> goal_handle)
  {
    std::thread{std::bind(&ScannerNode::execute, this, goal_handle)}.detach();
  }

  // --- Main Execution Logic ---
  void execute(const std::shared_ptr<GoalHandleScan> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), ">>> Scanner Activated. Stabilizing...");
    auto feedback = std::make_shared<ScanAction::Feedback>();
    
    // ==========================================================
    // 1. Stabilization Wait (2 seconds)
    // ==========================================================
    feedback->current_status = "Stabilizing Camera (Wait 2s)...";
    goal_handle->publish_feedback(feedback);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // ==========================================================

    RCLCPP_INFO(this->get_logger(), ">>> Capturing Data...");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ScanAction::Result>();

    // 2. Wait for fresh data (max 3 seconds)
    feedback->current_status = "Waiting for valid sensor data...";
    goal_handle->publish_feedback(feedback);
    
    bool data_ready = false;
    for(int i=0; i<30; i++) { // 30 * 100ms = 3sec
        {
            std::lock_guard<std::mutex> lock(img_mutex_);
            if(!latest_rgb_.empty() && !latest_depth_.empty()) {
                data_ready = true;
                break;
            }
        }
        std::this_thread::sleep_for(100ms);
    }

    if (!data_ready) {
        RCLCPP_ERROR(this->get_logger(), "Timeout: No Camera Data!");
        result->success = false;
        result->message = "No Image Data";
        goal_handle->abort(result);
        return;
    }

    // 3. Capture Data (Copy from shared variables)
    cv::Mat rgb_frame, depth_frame;
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        rgb_frame = latest_rgb_.clone();
        depth_frame = latest_depth_.clone();
    }

    // 4. Get Transform
    feedback->current_status = "Looking up TF...";
    goal_handle->publish_feedback(feedback);
    
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
        tf_msg = tf_buffer_.lookupTransform(world_frame_, camera_frame_, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
        result->success = false;
        result->message = "TF Lookup Failed";
        goal_handle->abort(result);
        return;
    }

    // 5. Save to Disk
    feedback->current_status = "Saving files...";
    goal_handle->publish_feedback(feedback);

    if (save_files(goal->label, rgb_frame, depth_frame, tf_msg)) {
        result->success = true;
        result->message = "Saved: " + goal->label;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "<<< Capture Success: %s", goal->label.c_str());
    } else {
        result->success = false;
        result->message = "File Write Error";
        goal_handle->abort(result);
    }
  }

  // --- Helper: File Saving ---
  bool save_files(const std::string& label, cv::Mat& rgb, cv::Mat& depth, const geometry_msgs::msg::TransformStamped& tf)
  {
    // ==========================================================
    // UPDATED: Logic for per-object counting (Object_0_1, Object_0_2...)
    // ==========================================================
    if (object_counters_.find(label) == object_counters_.end()) {
        object_counters_[label] = 0;
    }
    object_counters_[label]++; 
    int count = object_counters_[label]; 

    std::ostringstream base_name;
    base_name << label << "_" << count; // e.g. "Object_0_1"
    // ==========================================================

    std::string color_path = output_dir_ + "/color/" + base_name.str() + ".jpg";
    std::string depth_path = output_dir_ + "/depth/" + base_name.str() + ".png";
    std::string pose_path  = output_dir_ + "/poses/" + base_name.str() + ".txt";

    // A. Save RGB
    if(!cv::imwrite(color_path, rgb)) return false;

    // B. Save Depth (Process float -> uint16mm)
    cv::patchNaNs(depth, 0.0f);
    cv::threshold(depth, depth, 5.0, 0.0, cv::THRESH_TOZERO_INV); // Cutoff > 5m
    cv::Mat depth_u16;
    depth.convertTo(depth_u16, CV_16UC1, 1000.0);
    if(!cv::imwrite(depth_path, depth_u16)) return false;

    // C. Save Pose
    tf2::Quaternion q(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
    tf2::Matrix3x3 R(q);
    double x = tf.transform.translation.x;
    double y = tf.transform.translation.y;
    double z = tf.transform.translation.z;

    std::ofstream f(pose_path);
    if (!f.is_open()) return false;
    
    f << std::fixed << std::setprecision(6)
      << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << x << "\n"
      << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << y << "\n"
      << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << z << "\n"
      << "0.000000 0.000000 0.000000 1.000000\n";
    f.close();

    return true;
  }

  // --- Sensor Callbacks ---
  void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(img_mutex_);
    try {
      latest_rgb_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge RGB: %s", e.what());
    }
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(img_mutex_);
    try {
      // Keep as float for processing
      latest_depth_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge Depth: %s", e.what());
    }
  }

  // Members
  std::string rgb_topic_, depth_topic_, world_frame_, camera_frame_, output_dir_;
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  std::mutex img_mutex_; // Thread safety
  cv::Mat latest_rgb_;
  cv::Mat latest_depth_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScannerNode>());
  rclcpp::shutdown();
  return 0;
}