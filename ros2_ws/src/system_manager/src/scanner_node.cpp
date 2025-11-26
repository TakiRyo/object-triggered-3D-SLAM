#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <string> // <--- CHANGED: Needed for string

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "otslam_interfaces/action/scan_object.hpp"

using namespace std::chrono_literals;

class ScannerNode : public rclcpp::Node
{
public:
  using ScanAction = otslam_interfaces::action::ScanObject;
  using GoalHandleScan = rclcpp_action::ServerGoalHandle<ScanAction>;

  ScannerNode() : Node("scanner_node")
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<ScanAction>(
      this,
      "scan_object", 
      std::bind(&ScannerNode::handle_goal, this, _1, _2),
      std::bind(&ScannerNode::handle_cancel, this, _1),
      std::bind(&ScannerNode::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "=== 3D Scanner Action Server Ready ===");
  }

private:
  rclcpp_action::Server<ScanAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ScanAction::Goal> goal)
  {
    (void)uuid;
    // --- UPDATED: Print all info (Label + X + Y) ---
    RCLCPP_INFO(this->get_logger(), "Received Scan Request for: '%s' at (%.2f, %.2f)", 
        goal->label.c_str(), goal->x, goal->y);
    // -----------------------------------------------
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleScan> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Scan request canceled.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleScan> goal_handle)
  {
    std::thread{std::bind(&ScannerNode::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleScan> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), ">>> Scanning started... (Pretending to work)");

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ScanAction::Result>();
    auto feedback = std::make_shared<ScanAction::Feedback>();

    // --- UPDATED: Using String Feedback ---
    std::vector<std::string> steps = {"Aligning Sensors...", "Capturing PointCloud...", "Saving to Disk..."};

    for (const auto& step_name : steps) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Canceled";
        goal_handle->canceled(result);
        return;
      }
      
      // Send String Feedback
      feedback->current_status = step_name;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Status: %s", step_name.c_str());
      
      std::this_thread::sleep_for(1s);
    }
    // --------------------------------------

    if (rclcpp::ok()) {
      result->success = true;
      result->message = "Saved " + goal->label;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "<<< Scan Finished: %s", result->message.c_str());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScannerNode>());
  rclcpp::shutdown();
  return 0;
}