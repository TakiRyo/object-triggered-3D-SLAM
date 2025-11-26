#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <vector>
#include <limits>

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
    this->declare_parameter("visit_offset", 0.75);      
    this->declare_parameter("reach_threshold", 0.5);   
    visit_offset_ = this->get_parameter("visit_offset").as_double();
    reach_threshold_ = this->get_parameter("reach_threshold").as_double();

    cluster_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/stable_clusters", 10, std::bind(&GoalSender::clusterCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoalSender::odomCallback, this, std::placeholders::_1));

    // --- 修正1: トピック名を変更 (Nav2が反応しないようにする) ---
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/manager/target_pose", 10);
    // --------------------------------------------------------
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visiting_points", 10);

    RCLCPP_INFO(this->get_logger(), "✅ GoalSender Started (Topic: /manager/target_pose)");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
  }

  void clusterCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    // Step 1: Update clusters
    for (const auto &m : msg->markers)
    {
      float cx = m.pose.position.x;
      float cy = m.pose.position.y;
      float width = m.scale.x;
      float height = m.scale.y;
      
      bool exists = false;
      for (auto &c : clusters_) {
        if (std::hypot(cx - c.cx, cy - c.cy) < 0.3) { exists = true; break; }
      }
      if (exists) continue;

      ClusterVisitInfo cluster;
      cluster.id = clusters_.size();
      cluster.cx = cx; cluster.cy = cy;

      float min_x = cx - width/2.0; float max_x = cx + width/2.0;
      float min_y = cy - height/2.0; float max_y = cy + height/2.0;

      std::vector<std::pair<float, float>> raw_points = {
        {(min_x + max_x)/2, min_y},
        {(min_x + max_x)/2, max_y},
        {min_x, (min_y + max_y)/2},
        {max_x, (min_y + max_y)/2}
      };

      for (auto &p : raw_points) {
        float dx = p.first - cx;
        float dy = p.second - cy;
        float len = std::hypot(dx, dy);
        VisitPoint vp;
        vp.x = p.first + (dx / len) * visit_offset_;
        vp.y = p.second + (dy / len) * visit_offset_;
        vp.visited = false;
        cluster.visit_points.push_back(vp);
      }
      clusters_.push_back(cluster);
    }

    // Step 2: Check Status
    for (auto &c : clusters_)
    {
        bool cluster_complete = true;
        for (auto &vp : c.visit_points)
        {
            if (!vp.visited) {
                float dist = std::hypot(robot_x_ - vp.x, robot_y_ - vp.y);
                if (dist < reach_threshold_) {
                    vp.visited = true;
                    RCLCPP_INFO(this->get_logger(), "📍 Reached point in Cluster %d", c.id);
                }
            }
            if (!vp.visited) cluster_complete = false;
        }
        c.all_visited = cluster_complete;
    }

    // Step 3: Priority Logic
    VisitPoint *target_point = nullptr;
    ClusterVisitInfo *target_cluster = nullptr; // 中心座標計算用
    
    // Logic A: Active Cluster
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

    // Logic B: Global Search
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

    // Step 4: Publish Goal
    // if (target_point && target_cluster)
    // {
    //   // --- 修正2: 中心座標も渡して、向きを計算させる ---
    //   publishGoal(target_point, target_cluster->cx, target_cluster->cy);
    //   // -------------------------------------------
    // }

    // Step 4: Publish Goal
    if (target_point && target_cluster)
    {
      // ★ 修正: IDも渡す
      publishGoal(target_point, target_cluster->cx, target_cluster->cy, target_cluster->id);
    }

    // Step 5: Visualization
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

  // --- 修正3: 向きの計算ロジックを追加 ---
  void publishGoal(VisitPoint* p, float center_x, float center_y, int cluster_id) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = "map";
      goal.header.stamp = this->get_clock()->now();
      goal.pose.position.x = p->x;
      goal.pose.position.y = p->y;
      
      // ★ 修正: Z座標にIDを埋め込む (Hack!)
      goal.pose.position.z = (double)cluster_id; 
      // -------------------------------------

      float dx = center_x - p->x;
      float dy = center_y - p->y;
      float yaw = std::atan2(dy, dx);

      goal.pose.orientation.z = std::sin(yaw / 2.0);
      goal.pose.orientation.w = std::cos(yaw / 2.0);

      goal_pub_->publish(goal);
  }
  // ------------------------------------

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
            m.color.g = 1.0; m.color.a = 0.8; 
          } else if (c.id == active_cluster_id_) {
            m.color.r = 1.0; m.color.g = 1.0; m.color.a = 1.0; 
          } else {
            m.color.r = 1.0; m.color.a = 0.5;
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
  double visit_offset_, reach_threshold_;
  
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