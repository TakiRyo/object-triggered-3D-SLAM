#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <otslam_interfaces/action/scan_object.hpp>

using ScanAction = otslam_interfaces::action::ScanObject;
using GoalHandleScan = rclcpp_action::ClientGoalHandle<ScanAction>;

class SystemManager : public rclcpp::Node
{
public:
  SystemManager() : Node("system_manager")
  {
    // 1. Subscribe to LiDAR
    lidar_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/detected_object_poses", 10,
        std::bind(&SystemManager::lidar_callback, this, std::placeholders::_1));

    // 2. Action Client for Scanner
    scanner_client_ = rclcpp_action::create_client<ScanAction>(this, "scan_object");

    RCLCPP_INFO(this->get_logger(), "System Manager (C++) Ready.");
  }

private:
  bool is_busy_ = false;

  void lidar_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (is_busy_ || msg->poses.empty()) return;

    // Found Object!
    auto target = msg->poses[0];
    
    // Check if we should process it (Simple duplicate check logic can go here)
    start_scanning_task(target.position.x, target.position.y);
  }

  void start_scanning_task(float x, float y)
  {
    is_busy_ = true;

    if (!scanner_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Scanner Agent is OFFLINE.");
      is_busy_ = false;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Assigning Task: Scan Object at [%.2f, %.2f]", x, y);

    auto goal = ScanAction::Goal();
    goal.x = x;
    goal.y = y;
    goal.radius = 0.5;

    auto options = rclcpp_action::Client<ScanAction>::SendGoalOptions();
    options.result_callback = std::bind(&SystemManager::result_callback, this, std::placeholders::_1);
    
    scanner_client_->async_send_goal(goal, options);
  }

  void result_callback(const GoalHandleScan::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Task Finished: %s", result.result->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Task Failed.");
    }
    is_busy_ = false; // Ready for next object
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr lidar_sub_;
  rclcpp_action::Client<ScanAction>::SharedPtr scanner_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemManager>());
  rclcpp::shutdown();
  return 0;
}