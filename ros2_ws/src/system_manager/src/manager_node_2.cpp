#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "otslam_interfaces/action/scan_object.hpp"

// REMOVED: #include "../include/system_manager/goal_list.hpp" 

using namespace std::chrono_literals;

class SystemManager : public rclcpp::Node
{
public:
  using NavAction = nav2_msgs::action::NavigateToPose;
  using ScanAction = otslam_interfaces::action::ScanObject;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavAction>;
  using GoalHandleScan = rclcpp_action::ClientGoalHandle<ScanAction>;

  enum class State {
    IDLE,
    NAVIGATING,
    SCANNING
  };

  SystemManager() : Node("system_manager")
  {
    state_ = State::IDLE;

    // 1. Subscriber: Listen to GoalSender
    // We listen to /goal_pose, which sends "Where to go NOW"
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&SystemManager::goal_callback, this, std::placeholders::_1));

    // 2. Action Clients
    nav_client_ = rclcpp_action::create_client<NavAction>(this, "navigate_to_pose");
    scanner_client_ = rclcpp_action::create_client<ScanAction>(this, "scan_object");

    RCLCPP_INFO(this->get_logger(), "==========================================");
    RCLCPP_INFO(this->get_logger(), "   DYNAMIC SYSTEM MANAGER READY           ");
    RCLCPP_INFO(this->get_logger(), "   Waiting for /goal_pose...              ");
    RCLCPP_INFO(this->get_logger(), "==========================================");
  }

private:
  State state_;
  
  // Memory to check if the goal is new or the same as before
  double last_target_x_ = 0.0;
  double last_target_y_ = 0.0;
  bool first_goal_received_ = false;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
  rclcpp_action::Client<ScanAction>::SharedPtr scanner_client_;

  // --- Dynamic Goal Callback (Replaces the old Loop) ---
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Logic 1: If Robot is busy, ignore updates for now
    // (We finish the current task before accepting a new one)
    if (state_ != State::IDLE) {
        return; 
    }

    double new_x = msg->pose.position.x;
    double new_y = msg->pose.position.y;

    // Logic 2: Filter duplicates
    // GoalSender publishes constantly (e.g. 10Hz). We only react if the target changes.
    if (first_goal_received_) {
        double dist = std::hypot(new_x - last_target_x_, new_y - last_target_y_);
        if (dist < 0.05) {
            return; // Same goal, ignore.
        }
    }

    // Logic 3: Accept New Goal
    RCLCPP_INFO(this->get_logger(), ">>> NEW ORDER RECEIVED: (%.2f, %.2f)", new_x, new_y);
    
    last_target_x_ = new_x;
    last_target_y_ = new_y;
    first_goal_received_ = true;

    // Start Sequence
    send_nav_goal(msg);
  }

  // --- Navigation Handling ---
  void send_nav_goal(const geometry_msgs::msg::PoseStamped::SharedPtr target_msg)
  {
    if (!nav_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Nav2 Action Server not ready... waiting.");
      return; 
    }

    auto goal_msg = NavAction::Goal();
    goal_msg.pose = *target_msg; // Use the pose directly from GoalSender

    state_ = State::NAVIGATING;

    auto send_goal_options = rclcpp_action::Client<NavAction>::SendGoalOptions();
    send_goal_options.result_callback = 
      std::bind(&SystemManager::nav_result_callback, this, std::placeholders::_1);
    
    nav_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void nav_result_callback(const GoalHandleNav::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Arrived at target. Starting Scanner...");
      send_scan_goal();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Navigation Failed or Canceled. Retrying/Waiting...");
      state_ = State::IDLE; // Go back to IDLE to listen for the next goal
    }
  }

  // --- Scanner Handling ---
  void send_scan_goal()
  {
    if (!scanner_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(), "Scanner not ready. Skipping.");
      state_ = State::IDLE; 
      return;
    }

    auto goal_msg = ScanAction::Goal();
    
    // Auto-generate a label since GoalSender only sends coordinates
    // Format: "visit_X_Y"
    std::ostringstream label_ss;
    label_ss << "visit_" << std::fixed << std::setprecision(1) << last_target_x_ 
             << "_" << last_target_y_;
    
    goal_msg.label  = label_ss.str();
    goal_msg.x      = last_target_x_;
    goal_msg.y      = last_target_y_;
    goal_msg.radius = 1.0; 
    
    state_ = State::SCANNING;

    auto send_goal_options = rclcpp_action::Client<ScanAction>::SendGoalOptions();
    send_goal_options.result_callback = 
      std::bind(&SystemManager::scan_result_callback, this, std::placeholders::_1);

    scanner_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void scan_result_callback(const GoalHandleScan::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Task Finished. Ready for next order.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Scan Failed.");
    }

    // Critical: Go back to IDLE so we can accept the NEXT goal from GoalSender
    state_ = State::IDLE;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemManager>());
  rclcpp::shutdown();
  return 0;
}