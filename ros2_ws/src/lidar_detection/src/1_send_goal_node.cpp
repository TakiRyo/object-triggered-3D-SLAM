#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class Nav2GoalSender : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2GoalSender() : Node("nav2_goal_sender")
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    RCLCPP_INFO(this->get_logger(), "⏳ Waiting for Nav2 action server...");
    client_->wait_for_action_server();

    send_goal_timer_ = this->create_wall_timer(
      3s, std::bind(&Nav2GoalSender::sendGoal, this));
  }

private:
  void sendGoal()
  {
    send_goal_timer_->cancel(); // only send once

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();

    // 🟩 set your tentative goal (change these!)
    goal_msg.pose.pose.position.x = 1.5;
    goal_msg.pose.pose.position.y = -1.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "🚀 Sending goal: (%.2f, %.2f)",
                goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&Nav2GoalSender::resultCallback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, send_goal_options);
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
        RCLCPP_WARN(this->get_logger(), "Unknown result code.");
        break;
    }
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr send_goal_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2GoalSender>());
  rclcpp::shutdown();
  return 0;
}
