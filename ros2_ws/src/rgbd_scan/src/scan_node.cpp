#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <otslam_interfaces/action/scan_object.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <filesystem>
#include <mutex>

// Simplify types
using ScanAction = otslam_interfaces::action::ScanObject;
using NavAction  = nav2_msgs::action::NavigateToPose;
using GoalHandleScan = rclcpp_action::ServerGoalHandle<ScanAction>;

class ScannerNode : public rclcpp::Node
{
public:
  ScannerNode() : Node("scanner_node")
  {
    // 1. Action Server (Manager talks to this)
    scan_server_ = rclcpp_action::create_server<ScanAction>(
      this, "scan_object",
      std::bind(&ScannerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ScannerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ScannerNode::handle_accepted, this, std::placeholders::_1)
    );

    // 2. Action Client (Talks to Nav2)
    nav_client_ = rclcpp_action::create_client<NavAction>(this, "navigate_to_pose");

    // 3. Image Subscriber
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/intel_realsense_r200_rgb/image_raw", 10,
        std::bind(&ScannerNode::img_callback, this, std::placeholders::_1));

    // Config
    this->declare_parameter("save_path", "/home/ros2_env/taki/otslam/3d_model/data");
    save_path_ = this->get_parameter("save_path").as_string();
    std::filesystem::create_directories(save_path_);

    RCLCPP_INFO(this->get_logger(), "Scanner Agent Ready. Save Path: %s", save_path_.c_str());
  }

private:
  rclcpp_action::Server<ScanAction>::SharedPtr scan_server_;
  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  
  sensor_msgs::msg::Image::SharedPtr latest_img_;
  std::mutex img_mutex_;
  std::string save_path_;
  int photo_count_ = 0;

  // --- Image Callback ---
  void img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(img_mutex_);
    latest_img_ = msg;
  }

  // --- Action Handlers ---
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ScanAction::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleScan>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(const std::shared_ptr<GoalHandleScan> goal_handle) {
    std::thread{std::bind(&ScannerNode::execute, this, goal_handle)}.detach();
  }

  // --- MAIN LOGIC ---
  void execute(const std::shared_ptr<GoalHandleScan> goal_handle)
  {
    auto feedback = std::make_shared<ScanAction::Feedback>();
    auto result   = std::make_shared<ScanAction::Result>();
    auto goal     = goal_handle->get_goal();

    RCLCPP_INFO(this->get_logger(), "STARTING SCAN: Object at [%.2f, %.2f]", goal->x, goal->y);

    // 1. Generate 4 Viewpoints
    auto viewpoints = generate_circle_poses(goal->x, goal->y, 0.8); // 0.8m radius

    for (size_t i = 0; i < viewpoints.size(); ++i) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
      }

      // A. Update Status
      feedback->current_status = "Moving to Viewpoint " + std::to_string(i+1);
      goal_handle->publish_feedback(feedback);

      // B. Move Robot
      if (!move_robot_to(viewpoints[i])) {
        RCLCPP_WARN(this->get_logger(), "Failed to reach viewpoint %ld", i);
        continue; 
      }

      // C. Capture
      feedback->current_status = "Capturing Image...";
      goal_handle->publish_feedback(feedback);
      save_image_to_disk();
      
      // Wait a bit to simulate scanning
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    result->success = true;
    result->message = "Scan Complete";
    goal_handle->succeed(result);
  }

  bool move_robot_to(const geometry_msgs::msg::PoseStamped &target)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 not available!");
      return false;
    }

    auto goal = NavAction::Goal();
    goal.pose = target;

    auto future = nav_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) return false;

    auto goal_handle = future.get();
    if (!goal_handle) return false;

    auto result_future = nav_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) return false;

    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  void save_image_to_disk()
  {
    std::lock_guard<std::mutex> lock(img_mutex_);
    if (!latest_img_) return;

    try {
      cv::Mat cv_img = cv_bridge::toCvCopy(latest_img_, "bgr8")->image;
      std::string filename = save_path_ + "/img_" + std::to_string(photo_count_++) + ".jpg";
      cv::imwrite(filename, cv_img);
      RCLCPP_INFO(this->get_logger(), "Saved: %s", filename.c_str());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "CV Error");
    }
  }

  std::vector<geometry_msgs::msg::PoseStamped> generate_circle_poses(float cx, float cy, float r)
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    float angles[] = {0.0, 1.57, 3.14, 4.71}; // 0, 90, 180, 270 deg

    for (float ang : angles) {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "map";
      p.header.stamp = this->now();
      
      p.pose.position.x = cx + r * cos(ang);
      p.pose.position.y = cy + r * sin(ang);

      // Yaw: Face the center
      float yaw = atan2(cy - p.pose.position.y, cx - p.pose.position.x);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      p.pose.orientation.x = q.x();
      p.pose.orientation.y = q.y();
      p.pose.orientation.z = q.z();
      p.pose.orientation.w = q.w();

      poses.push_back(p);
    }
    return poses;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScannerNode>());
  rclcpp::shutdown();
  return 0;
}