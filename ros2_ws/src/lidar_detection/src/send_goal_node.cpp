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
  float min_x, max_x;
  float min_y, max_y;
  float cx, cy;
  float width, height;
  std::vector<VisitPoint> visit_points;
  bool all_visited = false;
};

class GoalSender : public rclcpp::Node
{
public:
  GoalSender() : Node("goal_sender")
  {
    // Parameters
    this->declare_parameter("visit_offset", 0.75);      // distance away from cluster
    this->declare_parameter("reach_threshold", 0.5);   // distance to mark visited
    visit_offset_ = this->get_parameter("visit_offset").as_double();
    reach_threshold_ = this->get_parameter("reach_threshold").as_double();

    // Subscribers & Publishers
    cluster_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/stable_clusters", 10, std::bind(&GoalSender::clusterCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoalSender::odomCallback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visiting_points", 10);

    RCLCPP_INFO(this->get_logger(), "✅ GoalSender node started.");
  }

private:
  // --- Callbacks ---
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
  }

  void clusterCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    // Step 1: Update clusters from stable markers
    for (const auto &m : msg->markers)
    {
      float cx = m.pose.position.x;
      float cy = m.pose.position.y;
      float width = m.scale.x;
      float height = m.scale.y;
      float min_x = cx - width / 2.0f;
      float max_x = cx + width / 2.0f;
      float min_y = cy - height / 2.0f;
      float max_y = cy + height / 2.0f;

      // Skip if this cluster already exists (near existing one)
      bool exists = false;
      for (auto &c : clusters_)
      {
        float dist = std::hypot(cx - c.cx, cy - c.cy);
        if (dist < 0.3) { exists = true; break; }
      }
      if (exists) continue;

      // Step 2: Create new cluster and generate 4 visiting points (midpoints of sides)
      ClusterVisitInfo cluster;
      cluster.cx = cx; cluster.cy = cy;
      cluster.min_x = min_x; cluster.max_x = max_x;
      cluster.min_y = min_y; cluster.max_y = max_y;
      cluster.width = width; cluster.height = height;

      // Midpoints (option B)
      std::vector<std::pair<float, float>> midpoints = {
        {(min_x + max_x)/2, min_y}, // bottom
        {(min_x + max_x)/2, max_y}, // top
        {min_x, (min_y + max_y)/2}, // left
        {max_x, (min_y + max_y)/2}  // right
      };

      for (auto &p : midpoints)
      {
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

    // Step 3: Choose nearest unvisited visiting point
    float min_dist = std::numeric_limits<float>::max();
    VisitPoint *target = nullptr;
    ClusterVisitInfo *target_cluster = nullptr;

    for (auto &c : clusters_)
    {
      for (auto &vp : c.visit_points)
      {
        if (vp.visited) continue;
        float dist = std::hypot(robot_x_ - vp.x, robot_y_ - vp.y);
        if (dist < min_dist)
        {
          min_dist = dist;
          target = &vp;
          target_cluster = &c;
        }
      }
    }

    // Step 4: Publish goal if found
    if (target && min_dist > 0.3)
    {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = "map";
      goal.header.stamp = this->get_clock()->now();
      goal.pose.position.x = target->x;
      goal.pose.position.y = target->y;
      goal.pose.orientation.w = 1.0;
      goal_pub_->publish(goal);

      RCLCPP_INFO(this->get_logger(),
                  "🎯 Sending goal to visiting point (%.2f, %.2f)", target->x, target->y);
    }

    // Step 5: Check reached points
    // for (auto &c : clusters_)
    // {
    //   bool all_done = true;
    //   for (auto &vp : c.visit_points)
    //   {
    //     float dist = std::hypot(robot_x_ - vp.x, robot_y_ - vp.y);
    //     if (dist < reach_threshold_)
    //       vp.visited = true;
    //     if (!vp.visited)
    //       all_done = false;
    //   }
    //   c.all_visited = all_done;
    // }
    // Step 5: Check if cluster visited
    for (auto &c : clusters_)
    {
        for (auto &vp : c.visit_points)
        {
            float dist = std::hypot(robot_x_ - vp.x, robot_y_ - vp.y);
            if (dist < reach_threshold_)
            {
            // Mark all visiting points in this cluster as visited
            for (auto &v : c.visit_points)
                v.visited = true;
            c.all_visited = true;
            RCLCPP_INFO(this->get_logger(), "✅ Cluster at (%.2f, %.2f) marked as visited.", c.cx, c.cy);
            break; // No need to check others
            }
        }
    }



    // Step 6: Visualize visiting points in RViz
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    for (auto &c : clusters_)
    {
      for (auto &vp : c.visit_points)
      {
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
          m.color.g = 1.0; // green = visited
          m.color.r = m.color.b = 0.0;
        } else {
          m.color.r = 1.0; // red = unvisited
          m.color.g = m.color.b = 0.0;
        }
        m.color.a = 0.8;
        markers.markers.push_back(m);
      }
    }
    marker_pub_->publish(markers);
  }

  // --- Members ---
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  double robot_x_ = 0.0, robot_y_ = 0.0;
  double visit_offset_, reach_threshold_;
  std::vector<ClusterVisitInfo> clusters_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSender>());
  rclcpp::shutdown();
  return 0;
}
