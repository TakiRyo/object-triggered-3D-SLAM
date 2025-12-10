/**
 * -----------------------------------------------------------------------
 * Node Name: GoalSender (Merged Version)
 * -----------------------------------------------------------------------
 * Logic:
 * 1. Listens to '/object_visiting_points' -> Stores in 'added_targets_'
 * 2. Listens to '/removed_object_visiting_points' -> Stores in 'removed_targets_'
 * 3. Control Loop (Timer): Merges lists -> Selects nearest -> Publishes Goal
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
#include <mutex>

struct TargetPoint {
    int unique_id;   
    int object_id;   
    float x, y;
    float qx, qy, qz, qw;
    std::string frame_id;
    bool is_removed_type; // True if from removed_object_selector
};

class GoalSender : public rclcpp::Node
{
public:
  GoalSender() : Node("goal_sender")
  {
    this->declare_parameter("reach_threshold", 0.60);
    reach_threshold_ = this->get_parameter("reach_threshold").as_double();

    // 1. Subscriber for ADDED Objects
    added_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/object_visiting_points", 10, 
      std::bind(&GoalSender::addedCallback, this, std::placeholders::_1));

    // 2. Subscriber for REMOVED Objects
    removed_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/removed_object_visiting_points", 10, 
      std::bind(&GoalSender::removedCallback, this, std::placeholders::_1));

    // 3. Odom
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoalSender::odomCallback, this, std::placeholders::_1));
    
    // 4. Publishers
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/manager/target_pose", 10);
    status_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/goal_status", 10);

    // 5. Timer loop (Control Loop) - 5Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&GoalSender::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "✅ GoalSender Ready. Merging Added & Removed targets.");
  }

private:
  // --- Member Variables ---
  std::vector<TargetPoint> added_targets_;
  std::vector<TargetPoint> removed_targets_;
  std::mutex data_mutex_;

  double robot_x_ = 0.0, robot_y_ = 0.0;
  double reach_threshold_;
  
  std::set<int> visited_ids_; // Stores IDs of visited added objects
  std::set<int> visited_removed_ids_; // Stores IDs of visited removed objects
  
  int active_target_id_ = -1;
  int current_object_focus_ = -1;
  TargetPoint active_point_; 

  // --- Callbacks ---

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
  }

  void addedCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      added_targets_.clear();
      parseMarkers(msg, added_targets_, false);
  }

  void removedCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      removed_targets_.clear();
      parseMarkers(msg, removed_targets_, true);
  }

  // Helper to parse marker array into TargetPoints
  void parseMarkers(const visualization_msgs::msg::MarkerArray::SharedPtr msg, 
                    std::vector<TargetPoint>& target_list, bool is_removed) 
  {
      for (const auto &m : msg->markers) {
          if (m.action != visualization_msgs::msg::Marker::ADD) continue;
          
          TargetPoint p;
          p.unique_id = m.id;
          p.is_removed_type = is_removed;
          
          // ID Scheme Parsing
          if (is_removed) {
              p.object_id = m.id - 5000; // Removed IDs start at 5000
          } else {
              p.object_id = m.id / 100;  // Added IDs are Obj*100 + k
          }

          p.frame_id = m.header.frame_id;
          p.x = m.pose.position.x;
          p.y = m.pose.position.y;
          p.qx = m.pose.orientation.x;
          p.qy = m.pose.orientation.y;
          p.qz = m.pose.orientation.z;
          p.qw = m.pose.orientation.w;

          // Only add if NOT visited
          bool visited = is_removed ? 
              (visited_removed_ids_.find(p.unique_id) != visited_removed_ids_.end()) :
              (visited_ids_.find(p.unique_id) != visited_ids_.end());

          if (!visited) {
              target_list.push_back(p);
          }
      }
  }

  // --- Main Control Loop (Decides Goal) ---
  void controlLoop() {
      std::lock_guard<std::mutex> lock(data_mutex_);

      // 1. Check if we reached the current active target
      if (active_target_id_ != -1) {
          float dist = std::hypot(robot_x_ - active_point_.x, robot_y_ - active_point_.y);
          
          if (dist < reach_threshold_) {
              RCLCPP_INFO(this->get_logger(), "📍 Reached Target %d (Obj %d)", active_target_id_, active_point_.object_id);
              
              if (active_point_.is_removed_type) {
                  visited_removed_ids_.insert(active_target_id_);
              } else {
                  visited_ids_.insert(active_target_id_);
              }
              active_target_id_ = -1;     
          }
      }

      // 2. Select NEXT Target if idle
      if (active_target_id_ == -1) {
          
          // Merge lists for selection
          std::vector<TargetPoint*> all_candidates;
          for (auto &t : added_targets_) all_candidates.push_back(&t);
          for (auto &t : removed_targets_) all_candidates.push_back(&t);

          if (!all_candidates.empty()) {
              TargetPoint* best_p = nullptr;
              float min_dist = std::numeric_limits<float>::max();

              // A. Sticky Logic: Finish current object first (only for Added objects usually)
              if (current_object_focus_ != -1) {
                  for (auto *t : all_candidates) {
                      if (t->object_id == current_object_focus_ && !t->is_removed_type) {
                          float d = std::hypot(robot_x_ - t->x, robot_y_ - t->y);
                          if (d < min_dist) { min_dist = d; best_p = t; }
                      }
                  }
              }

              // B. Global Search: Find closest ANY object
              if (best_p == nullptr) {
                  min_dist = std::numeric_limits<float>::max();
                  for (auto *t : all_candidates) {
                      float d = std::hypot(robot_x_ - t->x, robot_y_ - t->y);
                      if (d < min_dist) { min_dist = d; best_p = t; }
                  }
              }

              // C. Lock & Publish
              if (best_p) {
                  active_target_id_ = best_p->unique_id;
                  current_object_focus_ = best_p->object_id;
                  active_point_ = *best_p; 
                  
                  std::string type = active_point_.is_removed_type ? "REMOVED" : "ADDED";
                  RCLCPP_INFO(this->get_logger(), "🔒 New Goal: [%s] ID %d (Obj %d)", type.c_str(), active_target_id_, current_object_focus_);
                  
                  publishGoal(&active_point_);
              }
          }
      } else {
          // ★ Republish Goal Periodically (To ensure System Manager gets it)
          // Only republish if distance is significant to avoid spamming 'Arrival' logic
          float d = std::hypot(robot_x_ - active_point_.x, robot_y_ - active_point_.y);
          if (d > reach_threshold_) {
             publishGoal(&active_point_);
          }
      }

      // 3. Publish Status Markers
      publishStatusMarkers();
  }

  void publishGoal(TargetPoint* p) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = p->frame_id; 
      goal.header.stamp = this->get_clock()->now();
      
      goal.pose.position.x = p->x;
      goal.pose.position.y = p->y;
      goal.pose.position.z = 0.0; // Force 2D

      goal.pose.orientation.x = p->qx;
      goal.pose.orientation.y = p->qy;
      goal.pose.orientation.z = p->qz;
      goal.pose.orientation.w = p->qw;

      goal_pub_->publish(goal);
  }

  void publishStatusMarkers() {
      visualization_msgs::msg::MarkerArray status_array;
      
      auto addStatus = [&](const std::vector<TargetPoint>& list) {
          for (const auto &t : list) {
              visualization_msgs::msg::Marker s;
              s.header.frame_id = t.frame_id;
              s.header.stamp = this->now();
              s.ns = "status";
              s.id = t.unique_id;
              s.type = visualization_msgs::msg::Marker::SPHERE;
              s.action = visualization_msgs::msg::Marker::ADD;
              s.pose.position.x = t.x; s.pose.position.y = t.y; s.pose.position.z = 0.5;
              s.scale.x = 0.2; s.scale.y = 0.2; s.scale.z = 0.2; 
              
              if (t.unique_id == active_target_id_) {
                  s.color.r = 1.0; s.color.g = 0.0; s.color.b = 0.0; s.color.a = 1.0; // Red = Active
              } else {
                  s.color.r = 0.5; s.color.g = 0.5; s.color.b = 0.5; s.color.a = 0.5; // Gray = Waiting
              }
              status_array.markers.push_back(s);
          }
      };

      addStatus(added_targets_);
      addStatus(removed_targets_);
      status_pub_->publish(status_array);
  }

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr added_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr removed_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSender>());
  rclcpp::shutdown();
  return 0;
}