/**
 * -----------------------------------------------------------------------
 * Node Name: GoalSender
 * Status: FIXED
 * -----------------------------------------------------------------------
 * Fixes:
 * 1. Z-Height is now 0.0. (Nav2 will now accept "Arrival").
 * 2. ID Parsing matches the new Generator (divides by 100).
 * 3. Keeps Object ID logic internally so robot focuses on one object at a time.
 * -----------------------------------------------------------------------
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <vector>
#include <set>
#include <limits>
#include <algorithm>
#include <string>

struct TargetPoint {
    int unique_id;   // Marker ID (e.g., 101, 102)
    int object_id;   // Parent Object ID (e.g., 1)
    float x, y;
    float qx, qy, qz, qw;
    std::string frame_id;
    bool is_removed; // New field to distinguish between new and removed objects
};

class GoalSender : public rclcpp::Node
{
public:
  GoalSender() : Node("goal_sender")
  {
    this->declare_parameter("reach_threshold", 0.60);
    reach_threshold_ = this->get_parameter("reach_threshold").as_double();

    points_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/object_visiting_points", 10, 
      std::bind(&GoalSender::combinedCallback, this, std::placeholders::_1));

    removed_points_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/removed_object_visiting_points", 10, 
      std::bind(&GoalSender::combinedCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoalSender::odomCallback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/manager/target_pose", 10);
    status_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/goal_status", 10);

    RCLCPP_INFO(this->get_logger(), "✅ GoalSender Ready. Z-Height forced to 0.0 for Nav2.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    // Debug log to confirm odometry updates
    RCLCPP_INFO(this->get_logger(), "Robot Position: (%.2f, %.2f)", robot_x_, robot_y_);
  }

  void combinedCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    std::vector<TargetPoint> all_targets;

    for (const auto &m : msg->markers) {
        if (m.action != visualization_msgs::msg::Marker::ADD) continue;

        TargetPoint p;
        p.unique_id = m.id;
        p.object_id = m.id / 100; 
        p.frame_id = m.header.frame_id;
        p.x = m.pose.position.x;
        p.y = m.pose.position.y;
        p.qx = m.pose.orientation.x;
        p.qy = m.pose.orientation.y;
        p.qz = m.pose.orientation.z;
        p.qw = m.pose.orientation.w;
        p.is_removed = (msg->markers[0].ns == "removed_object_visiting_points");

        if (p.is_removed) {
            if (visited_removed_ids_.find(p.unique_id) == visited_removed_ids_.end()) {
                all_targets.push_back(p);
            }
        } else {
            if (visited_ids_.find(p.unique_id) == visited_ids_.end()) {
                all_targets.push_back(p);
            }
        }
    }

    if (active_target_id_ != -1) {
        float dist = std::hypot(robot_x_ - active_point_.x, robot_y_ - active_point_.y);

        RCLCPP_INFO(this->get_logger(), "Distance to Active Point: %.2f, Reach Threshold: %.2f", dist, reach_threshold_);

        if (dist < reach_threshold_) {
            RCLCPP_INFO(this->get_logger(), "📍 Reached Point ID %d (Obj %d)", active_target_id_, active_point_.object_id);

            if (active_point_.is_removed) {
                visited_removed_ids_.insert(active_target_id_);
            } else {
                visited_ids_.insert(active_target_id_);
            }

            active_target_id_ = -1;
            RCLCPP_INFO(this->get_logger(), "Active Target Cleared.");
        }
    }

    if (active_target_id_ == -1 && !all_targets.empty()) {
        TargetPoint* best_p = nullptr;
        float min_dist = std::numeric_limits<float>::max();

        for (auto &t : all_targets) {
            float d = std::hypot(robot_x_ - t.x, robot_y_ - t.y);
            if (d < min_dist) { min_dist = d; best_p = &t; }
        }

        if (best_p) {
            active_target_id_ = best_p->unique_id;
            active_point_ = *best_p;

            if (active_point_.is_removed) {
                RCLCPP_INFO(this->get_logger(), "🔒 Going to Removed Object Point %d (Obj %d)", active_target_id_, active_point_.object_id);
            } else {
                RCLCPP_INFO(this->get_logger(), "🔒 Going to New Object Point %d (Obj %d)", active_target_id_, active_point_.object_id);
            }

            publishGoal(&active_point_);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Visited IDs: %zu, Visited Removed IDs: %zu", visited_ids_.size(), visited_removed_ids_.size());
  }

  void publishGoal(TargetPoint* p) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = p->frame_id;
      goal.header.stamp = this->get_clock()->now();

      goal.pose.position.x = p->x;
      goal.pose.position.y = p->y;
      goal.pose.position.z = 0.0;

      goal.pose.orientation.x = p->qx;
      goal.pose.orientation.y = p->qy;
      goal.pose.orientation.z = p->qz;
      goal.pose.orientation.w = p->qw;

      // Debug log to print the full goal message
      RCLCPP_INFO(this->get_logger(), "Publishing Goal Message:\nPosition: [x: %.2f, y: %.2f, z: %.2f]\nOrientation: [x: %.2f, y: %.2f, z: %.2f, w: %.2f]",
                  goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                  goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);

      goal_pub_->publish(goal);
  }

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr points_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr removed_points_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr status_pub_;

  double robot_x_ = 0.0, robot_y_ = 0.0;
  double reach_threshold_;

  std::set<int> visited_ids_;
  std::set<int> visited_removed_ids_;
  int active_target_id_ = -1;
  TargetPoint active_point_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSender>());
  rclcpp::shutdown();
  return 0;
}