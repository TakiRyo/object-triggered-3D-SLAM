#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <vector>
#include <set>
#include <limits>
#include <algorithm>

struct TargetPoint {
    int unique_id;   // Marker ID
    int object_id;   // Parent Object ID
    float x, y;
    // Orientation (New!)
    float qx, qy, qz, qw;
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
      std::bind(&GoalSender::pointsCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoalSender::odomCallback, this, std::placeholders::_1));
    
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/manager/target_pose", 10);
    status_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/goal_status", 10);

    RCLCPP_INFO(this->get_logger(), "✅ GoalSender Ready. Reading Poses from Tracker.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
  }

  void pointsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    std::vector<TargetPoint> available_targets;

    // 1. Parse incoming markers
    for (const auto &m : msg->markers) {
        if (m.action != visualization_msgs::msg::Marker::ADD) continue;
        
        TargetPoint p;
        p.unique_id = m.id;
        p.object_id = m.id / 10; 
        p.x = m.pose.position.x;
        p.y = m.pose.position.y;
        // ★ Capture Orientation
        p.qx = m.pose.orientation.x;
        p.qy = m.pose.orientation.y;
        p.qz = m.pose.orientation.z;
        p.qw = m.pose.orientation.w;
        
        if (visited_ids_.find(p.unique_id) == visited_ids_.end()) {
            available_targets.push_back(p);
        }
    }

    // 2. Check if we reached the CURRENT ACTIVE target
    if (active_target_id_ != -1) {
        float dist = std::hypot(robot_x_ - active_point_.x, robot_y_ - active_point_.y);
        
        if (dist < reach_threshold_) {
            RCLCPP_INFO(this->get_logger(), "📍 Reached Point ID %d (Obj %d)", active_target_id_, active_target_id_/10);
            visited_ids_.insert(active_target_id_);
            active_target_id_ = -1;     
        }
    }

    // 3. Select NEXT Target
    if (active_target_id_ == -1 && !available_targets.empty()) {
        
        TargetPoint* best_p = nullptr;
        float min_dist = std::numeric_limits<float>::max();

        // A. Priority Search
        if (current_object_focus_ != -1) {
            for (auto &t : available_targets) {
                if (t.object_id == current_object_focus_) {
                    float d = std::hypot(robot_x_ - t.x, robot_y_ - t.y);
                    if (d < min_dist) { min_dist = d; best_p = &t; }
                }
            }
        }

        // B. Global Search
        if (best_p == nullptr) {
            min_dist = std::numeric_limits<float>::max();
            for (auto &t : available_targets) {
                float d = std::hypot(robot_x_ - t.x, robot_y_ - t.y);
                if (d < min_dist) { min_dist = d; best_p = &t; }
            }
        }

        // C. Lock
        if (best_p) {
            active_target_id_ = best_p->unique_id;
            current_object_focus_ = best_p->object_id;
            active_point_ = *best_p; 
            
            RCLCPP_INFO(this->get_logger(), "🔒 Locking onto Point %d (Obj %d)", active_target_id_, current_object_focus_);
        }
    }

    // 4. Publish Goal (Using the orientation from the tracker!)
    if (active_target_id_ != -1) {
        publishGoal(&active_point_);
    }
    
    publishStatusMarkers(msg);
  }

  void publishGoal(TargetPoint* p) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = "map";
      goal.header.stamp = this->get_clock()->now();
      goal.pose.position.x = p->x;
      goal.pose.position.y = p->y;
      goal.pose.position.z = (double)p->object_id; 

      // ★ Use the calculated orientation
      goal.pose.orientation.x = p->qx;
      goal.pose.orientation.y = p->qy;
      goal.pose.orientation.z = p->qz;
      goal.pose.orientation.w = p->qw;

      goal_pub_->publish(goal);
  }

  void publishStatusMarkers(const visualization_msgs::msg::MarkerArray::SharedPtr input_msg) {
      visualization_msgs::msg::MarkerArray status_array;
      for (const auto &m : input_msg->markers) {
          visualization_msgs::msg::Marker s = m;
          s.ns = "status";
          s.action = visualization_msgs::msg::Marker::ADD;
          s.scale.x = 0.2; s.scale.y = 0.2; s.scale.z = 0.2; 
          
          if (visited_ids_.count(s.id)) {
              s.color.r = 0.0; s.color.g = 1.0; s.color.b = 0.0; s.color.a = 1.0; 
          } else if (s.id == active_target_id_) {
              s.color.r = 1.0; s.color.g = 0.0; s.color.b = 0.0; s.color.a = 1.0; 
          } else {
              s.color.r = 0.5; s.color.g = 0.5; s.color.b = 0.5; s.color.a = 0.5; 
          }
          status_array.markers.push_back(s);
      }
      status_pub_->publish(status_array);
  }

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr points_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr status_pub_;

  double robot_x_ = 0.0, robot_y_ = 0.0;
  double reach_threshold_;
  
  std::set<int> visited_ids_;
  int active_target_id_ = -1;
  int current_object_focus_ = -1;
  TargetPoint active_point_; 
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSender>());
  rclcpp::shutdown();
  return 0;
}