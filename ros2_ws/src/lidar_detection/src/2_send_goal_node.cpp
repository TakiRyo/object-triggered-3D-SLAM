#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class Nav2GoalSender : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2GoalSender()
  : Node("nav2_goal_sender")
  {
    // Create action client for Nav2
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Subscribe to target goals from Lidar node
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_goal", 10,
      std::bind(&Nav2GoalSender::onGoalReceived, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "⏳ Waiting for Nav2 action server...");
    client_->wait_for_action_server();
    RCLCPP_INFO(this->get_logger(), "✅ Connected to Nav2! Waiting for /target_goal...");
  }

private:
  void onGoalReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "🎯 Received goal from detection: (x=%.2f, y=%.2f)",
      msg->pose.position.x, msg->pose.position.y);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = *msg;

    // Send goal asynchronously
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&Nav2GoalSender::resultCallback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "🚀 Sent goal to Nav2.");
  }

  void resultCallback(const GoalHandleNav::WrappedResult &result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "✅ Goal reached successfully!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(), "❌ Goal was aborted.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "⚠️ Goal was canceled.");
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "❓ Unknown result code.");
        break;
    }
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2GoalSender>());
  rclcpp::shutdown();
  return 0;
}
