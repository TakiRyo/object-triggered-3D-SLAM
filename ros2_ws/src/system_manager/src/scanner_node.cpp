#include <memory>
#include <thread>
#include <chrono>
#include <string>
#include <mutex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <map>

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

/*
 * Node Name: ScannerNode
 * Role: Data Capture Action Server
 * Functionality:
 * 1. Waits 2s (Stabilization)
 * 2. Captures Data
 * 3. Waits 2s (Post-Capture Cool down) <--- NEW
 * 4. Returns Success
 */

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

    std::filesystem::create_directories(output_dir_ + "/color");
    std::filesystem::create_directories(output_dir_ + "/depth");
    std::filesystem::create_directories(output_dir_ + "/poses");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
        rgb_topic_, qos, std::bind(&ScannerNode::rgb_callback, this, _1));
    
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, qos, std::bind(&ScannerNode::depth_callback, this, _1));

    action_server_ = rclcpp_action::create_server<ScanAction>(
      this,
      "scan_object", 
      std::bind(&ScannerNode::handle_goal, this, _1, _2),
      std::bind(&ScannerNode::handle_cancel, this, _1),
      std::bind(&ScannerNode::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "=== Real Scanner Ready (With Post-Wait) ===");
  }

private:
  std::map<std::string, int> object_counters_; 
  rclcpp_action::Server<ScanAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ScanAction::Goal> goal)
  {
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleScan> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleScan> goal_handle)
  {
    std::thread{std::bind(&ScannerNode::execute, this, goal_handle)}.detach();
  }

  // --- Main Execution Logic ---
  void execute(const std::shared_ptr<GoalHandleScan> goal_handle)
  {
    auto feedback = std::make_shared<ScanAction::Feedback>();
    auto result = std::make_shared<ScanAction::Result>();
    const auto goal = goal_handle->get_goal();
    
    // ==========================================================
    // 1. Stabilization Wait (2 seconds)
    // ==========================================================
    RCLCPP_INFO(this->get_logger(), "1. Stabilizing (2s)...");
    feedback->current_status = "Stabilizing (2s)...";
    goal_handle->publish_feedback(feedback);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ==========================================================
    // 2. Check Data & Capture
    // ==========================================================
    RCLCPP_INFO(this->get_logger(), "2. Capturing...");
    bool data_ready = false;
    for(int i=0; i<30; i++) {
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
        result->success = false;
        result->message = "No Image Data";
        goal_handle->abort(result);
        return;
    }

    cv::Mat rgb_frame, depth_frame;
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        rgb_frame = latest_rgb_.clone();
        depth_frame = latest_depth_.clone();
    }

    // ==========================================================
    // 3. TF Lookup & Save
    // ==========================================================
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
        tf_msg = tf_buffer_.lookupTransform(world_frame_, camera_frame_, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        result->success = false;
        result->message = "TF Lookup Failed";
        goal_handle->abort(result);
        return;
    }

    if (save_files(goal->label, rgb_frame, depth_frame, tf_msg)) {
        
        // ==========================================================
        // 4. POST-SCAN WAIT (2 seconds) <--- ADDED HERE
        // ==========================================================
        RCLCPP_INFO(this->get_logger(), "3. Post-Scan Cool Down (2s)...");
        feedback->current_status = "Post-Scan Wait (2s)...";
        goal_handle->publish_feedback(feedback);
        
        std::this_thread::sleep_for(std::chrono::seconds(2)); 
        // ==========================================================

        result->success = true;
        result->message = "Saved: " + goal->label;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "4. Task Complete.");
    } else {
        result->success = false;
        result->message = "File Write Error";
        goal_handle->abort(result);
    }
  }

  bool save_files(const std::string& label, cv::Mat& rgb, cv::Mat& depth, const geometry_msgs::msg::TransformStamped& tf)
  {
    if (object_counters_.find(label) == object_counters_.end()) {
        object_counters_[label] = 0;
    }
    object_counters_[label]++; 
    int count = object_counters_[label]; 

    std::ostringstream base_name;
    base_name << label << "_" << count;

    std::string color_path = output_dir_ + "/color/" + base_name.str() + ".jpg";
    std::string depth_path = output_dir_ + "/depth/" + base_name.str() + ".png";
    std::string pose_path  = output_dir_ + "/poses/" + base_name.str() + ".txt";

    if(!cv::imwrite(color_path, rgb)) return false;

    cv::patchNaNs(depth, 0.0f);
    cv::threshold(depth, depth, 5.0, 0.0, cv::THRESH_TOZERO_INV); 
    cv::Mat depth_u16;
    depth.convertTo(depth_u16, CV_16UC1, 1000.0);
    if(!cv::imwrite(depth_path, depth_u16)) return false;

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

  void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(img_mutex_);
    try { latest_rgb_ = cv_bridge::toCvCopy(msg, "bgr8")->image; } catch (...) {}
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(img_mutex_);
    try { latest_depth_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image; } catch (...) {}
  }

  std::string rgb_topic_, depth_topic_, world_frame_, camera_frame_, output_dir_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mutex img_mutex_;
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