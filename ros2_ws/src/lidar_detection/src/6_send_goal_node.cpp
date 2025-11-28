/*
 * Node Name: GoalSender
 * Role: Simple Sequencer
 * Logic:
 * 1. Listen to "/object_visiting_points" (Markers from Tracker).
 * 2. Find the nearest unvisited point.
 * 3. Send it to SystemManager.
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <vector>
#include <set>
#include <limits>

struct TargetPoint {
    int unique_id;   // Marker ID (ObjID * 10 + index)
    int object_id;   // Parent Object ID
    float x, y;
};

class GoalSender : public rclcpp::Node
{
public:
  GoalSender() : Node("goal_sender")
  {
    this->declare_parameter("reach_threshold", 0.60);
    reach_threshold_ = this->get_parameter("reach_threshold").as_double();

    // ★ Subscribe to the POINTS, not the objects
    points_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/object_visiting_points", 10, std::bind(&GoalSender::pointsCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoalSender::odomCallback, this, std::placeholders::_1));
    
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/manager/target_pose", 10);
    
    // We publish "Visited" markers (Green) to visualize progress
    status_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/goal_status", 10);

    RCLCPP_INFO(this->get_logger(), "✅ GoalSender Ready (Passive Mode).");
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
        p.object_id = m.id / 10; // Decode Parent ID
        p.x = m.pose.position.x;
        p.y = m.pose.position.y;
        
        // If we haven't visited it yet, add to list
        if (visited_ids_.find(p.unique_id) == visited_ids_.end()) {
            available_targets.push_back(p);
        } else {
            // Check if we need to "re-visit" (e.g. if robot drifted away?)
            // For now, assume visited = done forever.
        }
    }

    // 2. Check if we reached the CURRENT target
    if (active_target_id_ != -1) {
        // Find the active target coordinates
        bool target_still_exists = false;
        float tx=0, ty=0;
        
        // We look in the incoming message to find our active target's position
        for(const auto &t : available_targets) {
            if(t.unique_id == active_target_id_) {
                tx = t.x; ty = t.y; target_still_exists = true; break;
            }
        }

        if (target_still_exists) {
            float dist = std::hypot(robot_x_ - tx, robot_y_ - ty);
            if (dist < reach_threshold_) {
                RCLCPP_INFO(this->get_logger(), "📍 Reached Point ID %d (Obj %d)", active_target_id_, active_target_id_/10);
                visited_ids_.insert(active_target_id_);
                active_target_id_ = -1; // Done, ready for next
            }
        }
    }

    // 3. Select NEXT Target
    if (active_target_id_ == -1 && !available_targets.empty()) {
        
        // Logic: Find nearest point belonging to the CURRENT object (if any)
        // If no points left for current object, find nearest ANY object.
        
        TargetPoint* best_p = nullptr;
        float min_dist = std::numeric_limits<float>::max();

        // A. Priority Search: Try to finish current object first
        if (current_object_focus_ != -1) {
            for (auto &t : available_targets) {
                if (t.object_id == current_object_focus_) {
                    float d = std::hypot(robot_x_ - t.x, robot_y_ - t.y);
                    if (d < min_dist) { min_dist = d; best_p = &t; }
                }
            }
        }

        // B. Global Search: If no point found for current object (or no focus)
        if (best_p == nullptr) {
            for (auto &t : available_targets) {
                float d = std::hypot(robot_x_ - t.x, robot_y_ - t.y);
                if (d < min_dist) { min_dist = d; best_p = &t; }
            }
        }

        // C. Publish Goal
        if (best_p) {
            active_target_id_ = best_p->unique_id;
            current_object_focus_ = best_p->object_id;
            
            publishGoal(best_p);
            RCLCPP_INFO(this->get_logger(), "🔒 Locking onto Point %d (Obj %d)", active_target_id_, current_object_focus_);
        }
    }
    
    // 4. Visualize Status (Green = Visited, Red = Active)
    publishStatusMarkers(msg);
  }

  void publishGoal(TargetPoint* p) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = "map";
      goal.header.stamp = this->get_clock()->now();
      goal.pose.position.x = p->x;
      goal.pose.position.y = p->y;
      goal.pose.position.z = (double)p->object_id; // Send ObjID to Manager

      // Orientation: Face the center of the lock zone? 
      // GoalSender doesn't know the center anymore! 
      // Simple fix: Face the previous robot position? Or just face "inwards"?
      // For now: Just use 0 orientation or keep previous calculation.
      // Better: Include "Angle" in the Marker orientation from ObjectTracker!
      
      // Let's assume the ObjectTracker sets the Marker orientation to face the center.
      // We can't easily access that here without reading the quaternion from the marker message.
      // For simplicity, we send identity orientation, and let SystemManager/Scanner deal with it?
      // Or we calculate "face towards average of points"? 
      
      // *** Quick Hack: Just send valid orientation (0,0,0,1) for now ***
      goal.pose.orientation.w = 1.0; 

      goal_pub_->publish(goal);
  }

  void publishStatusMarkers(const visualization_msgs::msg::MarkerArray::SharedPtr input_msg) {
      visualization_msgs::msg::MarkerArray status_array;
      for (const auto &m : input_msg->markers) {
          visualization_msgs::msg::Marker s = m;
          s.ns = "status";
          s.action = visualization_msgs::msg::Marker::ADD;
          
          if (visited_ids_.count(s.id)) {
              s.color.r = 0.0; s.color.g = 1.0; s.color.b = 0.0; s.color.a = 1.0; // Green = Done
          } else if (s.id == active_target_id_) {
              s.color.r = 1.0; s.color.g = 0.0; s.color.b = 0.0; s.color.a = 1.0; // Red = Active
          } else {
              s.color.r = 0.5; s.color.g = 0.5; s.color.b = 0.5; s.color.a = 0.5; // Grey = Pending
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
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSender>());
  rclcpp::shutdown();
  return 0;
}