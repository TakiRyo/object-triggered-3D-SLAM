/**
 * -----------------------------------------------------------------------
 * Node Name: SystemManager
 * -----------------------------------------------------------------------
 * Purpose:
 * The central orchestrator (State Machine) for the "Move & Scan" mission.
 * It connects the Mission Manager (GoalSender), the Pilot (Nav2), 
 * the Photographer (ScannerNode), and the Tracker (ObjectClusterMarker).
 *
 * Role:
 * 1. Receives a target pose (Position + Orientation + Object ID).
 * 2. Navigates the robot to that pose.
 * 3. Upon arrival, triggers the precise data capture sequence.
 * 4. Manages the "Lidar vs. Camera" mode switching.
 *
 * Inputs:
 * /manager/target_pose (geometry_msgs::PoseStamped)
 * - From GoalSender. Contains the goal coordinates and the Object ID (in z).
 *
 * Actions & Services:
 * - Client: navigate_to_pose (Nav2) -> Moves the robot.
 * - Client: scan_object (ScannerNode) -> Captures Data.
 * - Client: set_tracking_mode (Service) -> Freezes/Unfreezes the object tracker.
 *
 * -----------------------------------------------------------------------
 * ★ SMART TRACKING STRATEGY (The "Freeze" Logic) ★
 * -----------------------------------------------------------------------
 * The critical logic in `goal_callback` handles when to trust the Lidar 
 * and when to trust the Memory (Frozen state):
 *
 * 1. NEW Object (ID changed):
 * Action: UNFREEZE (Enable Lidar Tracking).
 * Reason: We are approaching a new target. We need live Lidar data 
 * to find exactly where it is and center our markers.
 *
 * 2. SAME Object (ID unchanged, moving to next view):
 * Action: KEEP FROZEN.
 * Reason: We have already locked onto this object. As we move around 
 * it, parts of the object might become occluded, causing the center 
 * to shift wildly if we used live data. Keeping it frozen ensures 
 * all 4 visiting points remain relative to the *original* detected center.
 *
 * 3. ARRIVAL (Nav Success):
 * Action: FORCE FREEZE.
 * Reason: Ensure the coordinates are absolutely static during the 
 * 5-second camera stabilization phase.
 * -----------------------------------------------------------------------
 */

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
#include "std_srvs/srv/set_bool.hpp" 

using namespace std::chrono_literals;

class SystemManager : public rclcpp::Node
{
public:
  using NavAction = nav2_msgs::action::NavigateToPose;
  using ScanAction = otslam_interfaces::action::ScanObject;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavAction>;
  using GoalHandleScan = rclcpp_action::ClientGoalHandle<ScanAction>;
  using SetBoolService = std_srvs::srv::SetBool;

  enum class State {
    IDLE,
    NAVIGATING,
    SCANNING
  };

  SystemManager() : Node("system_manager")
  {
    state_ = State::IDLE;

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/manager/target_pose", 10, std::bind(&SystemManager::goal_callback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavAction>(this, "navigate_to_pose");
    scanner_client_ = rclcpp_action::create_client<ScanAction>(this, "scan_object");
    
    // Client for switching ObjectTracker mode
    tracking_client_ = this->create_client<SetBoolService>("set_tracking_mode");
    
    // Initial State: Enable Tracking to find the first object
    call_tracking_service(true); 

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
  rclcpp::Client<SetBoolService>::SharedPtr tracking_client_;

  // Initialize to -1 to ensure the first object (ID 0) triggers the "New Object" logic
  int current_obj_id_ = -1;

  // --- Freeze/Unfreeze Helper ---
  void call_tracking_service(bool enable_tracking)
  {
    if (!tracking_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Tracking Service not ready. Skipping switch.");
      return;
    }

    auto request = std::make_shared<SetBoolService::Request>();
    request->data = enable_tracking; 

    tracking_client_->async_send_request(request, 
      [this, enable_tracking](rclcpp::Client<SetBoolService>::SharedFuture future) {
        if (future.get()->success) {
          RCLCPP_INFO(this->get_logger(), "Tracking Mode Set: %s", 
            enable_tracking ? "**ENABLED (Lidar Search)**" : "**FROZEN (Camera Scan)**");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to set tracking mode.");
        }
      });
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (state_ != State::IDLE) return; 

    double new_x = msg->pose.position.x;
    double new_y = msg->pose.position.y;
    int new_id = (int)msg->pose.position.z;

    // --- CHECK FOR NEW OBJECT ---
    if (new_id != current_obj_id_) {
        // Different ID: We are done with the old object. 
        // UNFREEZE Lidar to search/confirm the new object.
        RCLCPP_INFO(this->get_logger(), "🔄 Switching Object (ID %d -> %d). Enabling Lidar Search.", current_obj_id_, new_id);
        call_tracking_service(true); 
    } else {
        // Same ID: We are moving to another view point of the same object.
        // KEEP FROZEN to ensure the center point doesn't shift.
        RCLCPP_INFO(this->get_logger(), "➡️ Moving to next point for Object %d. Keeping Lidar Frozen.", new_id);
    }
    // ----------------------------

    if (first_goal_received_) {
        double dist = std::hypot(new_x - last_target_x_, new_y - last_target_y_);
        if (dist < 0.05) return; 
    }

    RCLCPP_INFO(this->get_logger(), ">>> NEW ORDER: ID:%d at (%.2f, %.2f)", new_id, new_x, new_y);
    
    last_target_x_ = new_x;
    last_target_y_ = new_y;
    current_obj_id_ = new_id; 
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
      
      // ★ FREEZE Tracking upon arrival (Camera Time begins)
      // Whether we were searching or just moving, we must be stable now.
      call_tracking_service(false); 
      
      send_scan_goal();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Navigation Failed. Retrying...");
      // Do NOT unfreeze. Retry the exact same point.
      first_goal_received_ = false; 
      state_ = State::IDLE;
    }
  }

  void send_scan_goal()
  {
    if (!scanner_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(), "Scanner not ready.");
      state_ = State::IDLE; 
      // Only unfreeze if the scanner is broken, so we can try to move on
      call_tracking_service(true); 
      return;
    }

    auto goal_msg = ScanAction::Goal();
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
    
    // ★ KEY LOGIC CHANGE: 
    // We do NOT unfreeze here anymore.
    // We stay frozen until `goal_callback` sees a NEW Object ID.
    
    state_ = State::IDLE;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemManager>());
  rclcpp::shutdown();
  return 0;
}