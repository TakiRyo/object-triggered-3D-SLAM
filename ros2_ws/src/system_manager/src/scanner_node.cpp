#include <functional>
#include <memory>
#include <thread>
#include <chrono>

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

    // Action Serverの立ち上げ
    action_server_ = rclcpp_action::create_server<ScanAction>(
      this,
      "scan_object", // Managerが呼んでいる名前と一致させる
      std::bind(&ScannerNode::handle_goal, this, _1, _2),
      std::bind(&ScannerNode::handle_cancel, this, _1),
      std::bind(&ScannerNode::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "=== 3D Scanner Action Server Ready ===");
  }

private:
  rclcpp_action::Server<ScanAction>::SharedPtr action_server_;

  // 1. ゴール受信時のチェック
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ScanAction::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received Scan Request for: '%s'", goal->label.c_str());
    // 常に受け入れる
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 2. キャンセル要求時
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleScan> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Scan request canceled.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // 3. 実際の処理を開始
  void handle_accepted(const std::shared_ptr<GoalHandleScan> goal_handle)
  {
    // 重い処理でメインスレッドを止めないよう、別スレッドで実行
    std::thread{std::bind(&ScannerNode::execute, this, goal_handle)}.detach();
  }

  // 4. 実行ロジック (ここに将来カメラ処理を書く)
  void execute(const std::shared_ptr<GoalHandleScan> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), ">>> Scanning started... (Pretending to work)");

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ScanAction::Result>();
    auto feedback = std::make_shared<ScanAction::Feedback>();

    // 3秒間待機（スキャンのフリ）
    for (int i = 1; i <= 3; ++i) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Canceled";
        goal_handle->canceled(result);
        return;
      }
      
      // Feedback送信 (進捗状況)
      feedback->progress = i * 33.3f;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Scanning... %d/3s", i);
      
      std::this_thread::sleep_for(1s);
    }

    // 完了処理
    if (rclcpp::ok()) {
      result->success = true;
      result->message = "PointCloud Saved for " + goal->label;
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