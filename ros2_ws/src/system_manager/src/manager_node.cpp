#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "otslam_interfaces/action/scan_object.hpp"

#include "../include/system_manager/goal_list.hpp" 

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
    SCANNING,
    COMPLETED
  };

  SystemManager() : Node("system_manager")
  {
    goals_ = get_goal_list();
    current_goal_index_ = 0;
    state_ = State::IDLE;

    RCLCPP_INFO(this->get_logger(), "==========================================");
    RCLCPP_INFO(this->get_logger(), "       SYSTEM MANAGER INITIALIZED         ");
    RCLCPP_INFO(this->get_logger(), "==========================================");
    RCLCPP_INFO(this->get_logger(), "Loaded %zu goals from list:", goals_.size());

    for (const auto& goal : goals_) {
      RCLCPP_INFO(this->get_logger(), 
        " [ID:%d] Label: '%s' -> (X: %.2f, Y: %.2f, Theta: %.1f deg)", 
        goal.id, goal.label.c_str(), goal.x, goal.y, goal.theta);
    }
    RCLCPP_INFO(this->get_logger(), "==========================================");

    nav_client_ = rclcpp_action::create_client<NavAction>(this, "navigate_to_pose");
    scanner_client_ = rclcpp_action::create_client<ScanAction>(this, "scan_object");

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&SystemManager::control_loop, this));
  }

private:
  std::vector<GoalData> goals_;
  size_t current_goal_index_;
  State state_;

  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
  rclcpp_action::Client<ScanAction>::SharedPtr scanner_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void control_loop()
  {
    if (state_ == State::COMPLETED) return;

    if (state_ == State::IDLE) {
      if (current_goal_index_ < goals_.size()) {
        send_nav_goal(goals_[current_goal_index_]);
      } else {
        RCLCPP_INFO(this->get_logger(), "ALL TASKS COMPLETED! Resting...");
        state_ = State::COMPLETED;
      }
    }
  }

  void send_nav_goal(const GoalData & target)
  {
    // --- UPDATED: Better waiting logic ---
    if (!nav_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Nav2 Action Server not ready... waiting.");
      return; // Return here allows the timer to call this again in 1s
    }
    // -------------------------------------

    auto goal_msg = NavAction::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = target.x;
    goal_msg.pose.pose.position.y = target.y;
    
    double rad = target.theta * (M_PI / 180.0);
    goal_msg.pose.pose.orientation.z = sin(rad / 2.0);
    goal_msg.pose.pose.orientation.w = cos(rad / 2.0);

    RCLCPP_INFO(this->get_logger(), ">>> STARTING Task %d: Go to '%s'", target.id, target.label.c_str());

    state_ = State::NAVIGATING;

    auto send_goal_options = rclcpp_action::Client<NavAction>::SendGoalOptions();
    send_goal_options.result_callback = 
      std::bind(&SystemManager::nav_result_callback, this, std::placeholders::_1);
    
    nav_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void nav_result_callback(const GoalHandleNav::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Navigation Succeeded. Starting Scanner...");
      send_scan_goal(goals_[current_goal_index_]);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Navigation Failed/Canceled. Skipping goal.");
      state_ = State::IDLE;
      current_goal_index_++; 
    }
  }

  void send_scan_goal(const GoalData & target)
  {
    if (!scanner_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(), "Scanner not ready. Skipping scan, marking as done.");
      state_ = State::IDLE;
      current_goal_index_++;
      return;
    }

    auto goal_msg = ScanAction::Goal();
    
    // --- UPDATED: Fill in the new Action Goal fields ---
    goal_msg.label  = target.label; 
    goal_msg.x      = target.x;
    goal_msg.y      = target.y;
    goal_msg.radius = 1.0; // Default radius
    // --------------------------------------------------
    
    state_ = State::SCANNING;

    auto send_goal_options = rclcpp_action::Client<ScanAction>::SendGoalOptions();
    send_goal_options.result_callback = 
      std::bind(&SystemManager::scan_result_callback, this, std::placeholders::_1);

    scanner_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void scan_result_callback(const GoalHandleScan::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Scan Completed successfully: %s", result.result->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Scan Failed.");
    }

    state_ = State::IDLE;
    current_goal_index_++;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemManager>());
  rclcpp::shutdown();
  return 0;
}