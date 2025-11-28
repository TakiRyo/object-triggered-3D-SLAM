/*
 * Node Name: GoalSender
 * Role: Select nearest object and generate visiting points DYNAMICALLY.
 * * LOGIC UPDATE:
 * - Instead of a fixed offset (e.g. 1.5m), we calculate the "Lock Zone" radius
 * based on the object size (Width/Height) + Margin.
 * - Visiting Points are placed on the perimeter of this circle.
 * - Result: Robot stands close to small objects, and far from large objects.
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

struct VisitPoint {
  float x, y;
  bool visited;
};

struct ClusterVisitInfo {
  int id;
  float cx, cy;
  std::vector<VisitPoint> visit_points;
  bool all_visited = false;
};

class GoalSender : public rclcpp::Node
{
public:
  GoalSender() : Node("goal_sender")
  {
    // --- Parameters ---
    
    // 1. REACH THRESHOLD (Must be > Nav2 tolerance of 0.25)
    //    We use 0.60 to ensure GoalSender is "satisfied" easily once Nav2 stops.
    this->declare_parameter("reach_threshold", 0.60);
    
    // 2. LOCK MARGIN (Must match ObjectTracker's margin)
    //    This adds padding to the object size to define the "Red Circle".
    this->declare_parameter("lock_margin", 0.5); 

    // 3. EXTRA BUFFER
    //    How far OUTSIDE the Red Circle should the robot stand?
    //    0.0 = On the line. 0.2 = 20cm outside the line (Safe).
    this->declare_parameter("extra_buffer", 0.2); 

    reach_threshold_ = this->get_parameter("reach_threshold").as_double();
    lock_margin_     = this->get_parameter("lock_margin").as_double();
    extra_buffer_    = this->get_parameter("extra_buffer").as_double();

    // Subscribe to STABLE OBJECTS (Green markers)
    cluster_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/stable_clusters", 10, std::bind(&GoalSender::clusterCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoalSender::odomCallback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/manager/target_pose", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visiting_points", 10);

    RCLCPP_INFO(this->get_logger(), "✅ GoalSender Ready.");
    RCLCPP_INFO(this->get_logger(), "   Method: Dynamic Lock Zone (Margin: %.2f + Buffer: %.2f)", lock_margin_, extra_buffer_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
  }

  void clusterCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    for (const auto &m : msg->markers)
    {
      if (m.action != visualization_msgs::msg::Marker::ADD) continue;

      int incoming_id = m.id;

      // Check if ID exists (First Sight Lock)
      bool already_exists = false;
      for(const auto& c : clusters_) {
          if(c.id == incoming_id) {
              already_exists = true;
              break;
          }
      }
      if (already_exists) continue; 

      // --- NEW: Calculate Dynamic Radius ---
      float cx = m.pose.position.x;
      float cy = m.pose.position.y;
      float width = m.scale.x;
      float height = m.scale.y;

      ClusterVisitInfo cluster;
      cluster.id = incoming_id;
      cluster.cx = cx; cluster.cy = cy;

      // Formula: Radius = (Diagonal / 2) + Lock Margin + Extra Buffer
      float diagonal = std::sqrt(width*width + height*height);
      float target_radius = (diagonal / 2.0f) + lock_margin_ + extra_buffer_;

      RCLCPP_INFO(this->get_logger(), "🆕 ID %d (Size %.2fx%.2f) -> Visit Radius: %.2fm", 
        incoming_id, width, height, target_radius);

      // Generate 4 Points on the Circle
      std::vector<std::pair<float, float>> raw_points = {
        {cx, cy + target_radius}, // North
        {cx, cy - target_radius}, // South
        {cx + target_radius, cy}, // East
        {cx - target_radius, cy}  // West
      };

      for (auto &p : raw_points) {
        VisitPoint vp;
        vp.x = p.first;
        vp.y = p.second;
        vp.visited = false;
        cluster.visit_points.push_back(vp);
      }
      clusters_.push_back(cluster);
    }

    // --- Check Status ---
    for (auto &c : clusters_)
    {
        bool cluster_complete = true;
        for (auto &vp : c.visit_points)
        {
            if (!vp.visited) {
                float dist = std::hypot(robot_x_ - vp.x, robot_y_ - vp.y);
                
                // Use the loose threshold (0.60m) to confirm arrival
                if (dist < reach_threshold_) {
                    vp.visited = true;
                    RCLCPP_INFO(this->get_logger(), "📍 Reached point in Cluster %d", c.id);
                }
            }
            if (!vp.visited) cluster_complete = false;
        }
        c.all_visited = cluster_complete;
    }

    // --- Priority Logic ---
    VisitPoint *target_point = nullptr;
    ClusterVisitInfo *target_cluster = nullptr;
    
    // Logic A: Finish Active Cluster
    if (active_cluster_id_ != -1) {
        for(auto &c : clusters_) { if(c.id == active_cluster_id_) target_cluster = &c; }

        if (target_cluster && !target_cluster->all_visited) {
            target_point = getNearestUnvisitedInCluster(target_cluster);
        } else {
            RCLCPP_INFO(this->get_logger(), "🔓 Cluster %d finished.", active_cluster_id_);
            active_cluster_id_ = -1; 
            target_cluster = nullptr;
        }
    }

    // Logic B: Find New Cluster
    if (active_cluster_id_ == -1) {
        float min_global_dist = std::numeric_limits<float>::max();

        for (auto &c : clusters_) {
            if (c.all_visited) continue;
            float d = std::hypot(robot_x_ - c.cx, robot_y_ - c.cy);
            if (d < min_global_dist) {
                min_global_dist = d;
                target_cluster = &c;
            }
        }

        if (target_cluster) {
            active_cluster_id_ = target_cluster->id;
            target_point = getNearestUnvisitedInCluster(target_cluster);
            RCLCPP_INFO(this->get_logger(), "🔒 Locked onto Cluster ID %d", active_cluster_id_);
        }
    }

    // Publish Goal
    if (target_point && target_cluster)
    {
      publishGoal(target_point, target_cluster->cx, target_cluster->cy, target_cluster->id);
    }

    // Visualize
    publishMarkers();
  }

  // --- Helpers ---
  VisitPoint* getNearestUnvisitedInCluster(ClusterVisitInfo* cluster) {
      float min_dist = std::numeric_limits<float>::max();
      VisitPoint* best_p = nullptr;
      for (auto &vp : cluster->visit_points) {
          if (vp.visited) continue;
          float d = std::hypot(robot_x_ - vp.x, robot_y_ - vp.y);
          if (d < min_dist) { min_dist = d; best_p = &vp; }
      }
      return best_p;
  }

  void publishGoal(VisitPoint* p, float center_x, float center_y, int cluster_id) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = "map";
      goal.header.stamp = this->get_clock()->now();
      goal.pose.position.x = p->x;
      goal.pose.position.y = p->y;
      
      // ID Embedded in Z
      goal.pose.position.z = (double)cluster_id; 

      float dx = center_x - p->x;
      float dy = center_y - p->y;
      float yaw = std::atan2(dy, dx);

      goal.pose.orientation.z = std::sin(yaw / 2.0);
      goal.pose.orientation.w = std::cos(yaw / 2.0);

      goal_pub_->publish(goal);
  }

  void publishMarkers() {
      visualization_msgs::msg::MarkerArray markers;
      int id = 0;
      for (auto &c : clusters_) {
        for (auto &vp : c.visit_points) {
          visualization_msgs::msg::Marker m;
          m.header.frame_id = "map";
          m.header.stamp = this->get_clock()->now();
          m.ns = "visiting_points";
          m.id = id++;
          m.type = visualization_msgs::msg::Marker::SPHERE;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.pose.position.x = vp.x;
          m.pose.position.y = vp.y;
          m.pose.position.z = 0.1;
          m.scale.x = m.scale.y = m.scale.z = 0.15;
          if (vp.visited) {
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8; 
          } else if (c.id == active_cluster_id_) {
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0; 
          } else {
            m.color.r = 0.5; m.color.g = 0.5; m.color.b = 0.5; m.color.a = 0.5;
          }
          markers.markers.push_back(m);
        }
      }
      marker_pub_->publish(markers);
  }

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  double robot_x_ = 0.0, robot_y_ = 0.0;
  double reach_threshold_, lock_margin_, extra_buffer_;
  
  std::vector<ClusterVisitInfo> clusters_;
  int active_cluster_id_ = -1; 
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSender>());
  rclcpp::shutdown();
  return 0;
}