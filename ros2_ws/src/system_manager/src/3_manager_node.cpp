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

    // --- 修正1: トピック名を変更 ---
    // GoalSenderが出す "/manager/target_pose" を受け取る
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/manager/target_pose", 10, std::bind(&SystemManager::goal_callback, this, std::placeholders::_1));
    // ----------------------------

    nav_client_ = rclcpp_action::create_client<NavAction>(this, "navigate_to_pose");
    scanner_client_ = rclcpp_action::create_client<ScanAction>(this, "scan_object");

    RCLCPP_INFO(this->get_logger(), "=== DYNAMIC SYSTEM MANAGER READY ===");
  }

private:
  State state_;
  double last_target_x_ = 0.0;
  double last_target_y_ = 0.0;
  bool first_goal_received_ = false;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
  rclcpp_action::Client<ScanAction>::SharedPtr scanner_client_;

  // メンバ変数に追加
  int current_obj_id_ = 0; // ID保存用

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (state_ != State::IDLE) return; 

    double new_x = msg->pose.position.x;
    double new_y = msg->pose.position.y;
    // ★ 追加: ZからIDを取り出す
    int new_id = (int)msg->pose.position.z;

    if (first_goal_received_) {
        double dist = std::hypot(new_x - last_target_x_, new_y - last_target_y_);
        if (dist < 0.05) return; 
    }

    RCLCPP_INFO(this->get_logger(), ">>> NEW ORDER: ID:%d at (%.2f, %.2f)", new_id, new_x, new_y);
    
    last_target_x_ = new_x;
    last_target_y_ = new_y;
    current_obj_id_ = new_id; // 保存
    first_goal_received_ = true;

    send_nav_goal(msg);
  }

  void send_nav_goal(const geometry_msgs::msg::PoseStamped::SharedPtr target_msg)
  {
    if (!nav_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Nav2 Action Server not ready.");
      return; 
    }

    auto goal_msg = NavAction::Goal();
    goal_msg.pose = *target_msg; 

    state_ = State::NAVIGATING;

    auto send_goal_options = rclcpp_action::Client<NavAction>::SendGoalOptions();
    send_goal_options.result_callback = 
      std::bind(&SystemManager::nav_result_callback, this, std::placeholders::_1);
    
    nav_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void nav_result_callback(const GoalHandleNav::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Arrived. Starting Scanner...");
      send_scan_goal();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Navigation Failed. Retrying...");
      
      // --- 修正2: 失敗したら記憶を消して再送を受け付ける ---
      first_goal_received_ = false; 
      // ----------------------------------------------
      
      state_ = State::IDLE;
    }
  }

  void send_scan_goal()
  {
    // if (!scanner_client_->wait_for_action_server(2s)) {
    //   RCLCPP_WARN(this->get_logger(), "Scanner not ready.");
    //   state_ = State::IDLE; 
    //   return;
    // }

    // auto goal_msg = ScanAction::Goal();
    // std::ostringstream label_ss;
    // label_ss << "obj_" << std::fixed << std::setprecision(1) << last_target_x_ 
    //          << "_" << last_target_y_;
    
    // goal_msg.label  = label_ss.str();
    // goal_msg.x      = last_target_x_;
    // goal_msg.y      = last_target_y_;
    // goal_msg.radius = 1.0; 
    
    // state_ = State::SCANNING;

    if (!scanner_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(), "Scanner not ready.");
      state_ = State::IDLE; 
      return;
    }

    auto goal_msg = ScanAction::Goal();
    
    // ★ 修正: ラベルを "Object_{ID}" にする
    // Scanner側で連番を振るので、ここでは物体名だけでOK
    std::ostringstream label_ss;
    label_ss << "Object_" << current_obj_id_;
    
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
      RCLCPP_INFO(this->get_logger(), "Task Finished.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Scan Failed.");
    }
    state_ = State::IDLE;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemManager>());
  rclcpp::shutdown();
  return 0;
}