// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/point_cloud2_iterator.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <cmath>
// #include <vector>

// // Helper for simple Euclidean clustering (same as in multi_object_goal_selector)
// struct PointXY { float x, y; };

// class RemovedObjectGoalSelector : public rclcpp::Node
// {
// public:
//   RemovedObjectGoalSelector() : Node("removed_object_goal_selector")
//   {
//     // --- Parameters (Only what's needed for clustering) ---
//     this->declare_parameter("cluster_distance_threshold", 0.4);
//     this->declare_parameter("min_cluster_points", 8);          

//     cluster_dist_thresh_ = this->get_parameter("cluster_distance_threshold").as_double();
//     min_cluster_pts_     = this->get_parameter("min_cluster_points").as_int();

//     // --- Subscribers ---
//     // Subscribes to removed objects from diff_node.cpp
//     removed_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//       "/removed_objects", 10,
//       std::bind(&RemovedObjectGoalSelector::removedCallback, this, std::placeholders::_1));

//     // --- Publisher ---
//     // Publishes goals for removed objects to a dedicated topic
//     goal_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/removed_object_visiting_points", 10);

//     RCLCPP_INFO(this->get_logger(), "Removed Object Goal Selector Ready.");
//   }

// private:
//   // Parameters
//   double cluster_dist_thresh_;
//   int min_cluster_pts_;

//   // Callbacks
//   void removedCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
//     if (msg->data.empty()) {
//         // Clear old markers if no objects are present
//         visualization_msgs::msg::MarkerArray clear_markers;
//         visualization_msgs::msg::Marker clear_marker;
//         clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
//         clear_markers.markers.push_back(clear_marker);
//         goal_pub_->publish(clear_markers);
//         return;
//     }

//     auto clusters = extractClusters(msg);
//     visualization_msgs::msg::MarkerArray markers;

//     for (size_t i = 0; i < clusters.size(); ++i) {
//         // Core Logic: Generate ONE checking point AT the center
//         generateSinglePoint(clusters[i], i, markers);
//     }
    
//     goal_pub_->publish(markers);
//   }

//   // --- Logic for REMOVED Objects (Check Center) ---
//   void generateSinglePoint(const std::vector<PointXY>& cluster, int obj_id, visualization_msgs::msg::MarkerArray &markers) {
//       if (cluster.empty()) return;

//       // Compute Center (Centroid)
//       float cx = 0, cy = 0;
//       for (auto& p : cluster) { cx += p.x; cy += p.y; }
//       cx /= cluster.size(); cy /= cluster.size();

//       visualization_msgs::msg::Marker m;
//       m.header.frame_id = "map"; // Assuming global frame is 'map'
//       m.header.stamp = this->now();
//       m.ns = "removed_goals_check_point";
      
//       // Use a large ID range (e.g., 5000+) to prevent conflict if merged later
//       m.id = 5000 + obj_id; 

//       m.type = visualization_msgs::msg::Marker::SPHERE; 
//       m.action = visualization_msgs::msg::Marker::ADD;
//       m.pose.position.x = cx;
//       m.pose.position.y = cy;
//       m.pose.orientation.w = 1.0;

//       m.scale.x = 0.3; m.scale.y = 0.3; m.scale.z = 0.3;
//       m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 1.0; // Purple for 'removed'

//       markers.markers.push_back(m);
//   }

//   // --- Helper: Cluster Points (Standard DBSCAN-like approach) ---
//   std::vector<std::vector<PointXY>> extractClusters(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
//       std::vector<std::vector<PointXY>> clusters;
//       std::vector<PointXY> points;
      
//       sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
//       sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");

//       for (; in_x != in_x.end(); ++in_x, ++in_y) {
//           points.push_back({*in_x, *in_y});
//       }

//       std::vector<bool> visited(points.size(), false);
//       for (size_t i = 0; i < points.size(); ++i) {
//           if (visited[i]) continue;
          
//           std::vector<PointXY> current_cluster;
//           current_cluster.push_back(points[i]);
//           visited[i] = true;

//           for (size_t j = 0; j < current_cluster.size(); ++j) {
//               PointXY p = current_cluster[j];
//               for (size_t k = 0; k < points.size(); ++k) {
//                   if (visited[k]) continue;
//                   float dist = std::hypot(p.x - points[k].x, p.y - points[k].y);
//                   if (dist < cluster_dist_thresh_) {
//                       visited[k] = true;
//                       current_cluster.push_back(points[k]);
//                   }
//               }
//           }
//           if (current_cluster.size() >= (size_t)min_cluster_pts_) {
//               clusters.push_back(current_cluster);
//           }
//       }
//       return clusters;
//   }

//   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr removed_sub_;
//   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_pub_;
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<RemovedObjectGoalSelector>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

// Simple point structure for clustering
struct PointXY { float x, y; };

// Candidate Goal Wrapper
struct CandidateGoal {
    geometry_msgs::msg::PointStamped goal_point;
    double distance_to_robot;
    int cluster_id;
};

class RemovedObjectGoalSelector : public rclcpp::Node
{
public:
  RemovedObjectGoalSelector() : Node("removed_object_goal_selector")
  {
    // --- Parameters ---
    this->declare_parameter("cluster_distance_threshold", 0.4);
    this->declare_parameter("min_cluster_points", 4);          
    this->declare_parameter("goal_distance_from_center", 0.6); // Radius of "Visiting Circle"
    this->declare_parameter("num_goal_points_per_cluster", 8); 
    this->declare_parameter("robot_base_frame", "base_link"); 
    this->declare_parameter("map_frame", "map");             

    cluster_dist_thresh_   = this->get_parameter("cluster_distance_threshold").as_double();
    min_cluster_pts_       = this->get_parameter("min_cluster_points").as_int();
    goal_dist_from_center_ = this->get_parameter("goal_distance_from_center").as_double();
    num_goal_points_       = this->get_parameter("num_goal_points_per_cluster").as_int();
    robot_base_frame_      = this->get_parameter("robot_base_frame").as_string();
    map_frame_             = this->get_parameter("map_frame").as_string();

    // --- TF Setup (Shared Ptr style like your other node) ---
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- Subscriber ---
    removed_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/removed_objects", 10,
      std::bind(&RemovedObjectGoalSelector::removedCallback, this, std::placeholders::_1));

    // --- Publisher ---
    goal_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/removed_object_visiting_points", 10);

    RCLCPP_INFO(this->get_logger(), "✅ Removed Object Goal Selector Ready.");
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr removed_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_pub_;

  double cluster_dist_thresh_;
  int min_cluster_pts_;
  double goal_dist_from_center_;
  int num_goal_points_;
  std::string robot_base_frame_;
  std::string map_frame_;

  void removedCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1. Clear if empty
    if (msg->data.empty()) {
        publishClearMarkers();
        return;
    }

    // 2. Check TF Availability (Prevents Warning Spam)
    if (!tf_buffer_->canTransform(map_frame_, robot_base_frame_, tf2::TimePointZero)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "Waiting for TF: %s -> %s", map_frame_.c_str(), robot_base_frame_.c_str());
        return;
    }

    // 3. Get Robot Pose
    geometry_msgs::msg::TransformStamped robot_transform;
    try {
        robot_transform = tf_buffer_->lookupTransform(
            map_frame_, robot_base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        return;
    }
    double robot_x = robot_transform.transform.translation.x;
    double robot_y = robot_transform.transform.translation.y;

    // 4. Cluster Points
    auto clusters = extractClusters(msg);
    if (clusters.empty()) {
        publishClearMarkers();
        return;
    }

    // 5. Generate Goals & Find Nearest
    std::vector<CandidateGoal> candidates;

    for (size_t i = 0; i < clusters.size(); ++i) {
        // Generate circular points around this cluster center
        auto circular_goals = generateCircularGoals(clusters[i], msg->header.stamp);
        
        for (auto& g : circular_goals) {
            double d = std::hypot(g.point.x - robot_x, g.point.y - robot_y);
            candidates.push_back({g, d, (int)i});
        }
    }

    if (candidates.empty()) return;

    // Sort by distance (Nearest first)
    std::sort(candidates.begin(), candidates.end(), 
        [](const CandidateGoal& a, const CandidateGoal& b) {
            return a.distance_to_robot < b.distance_to_robot;
        });

    // 6. Publish The Winner
    // We select candidates[0] which is the single nearest point across ALL removed objects.
    visualization_msgs::msg::MarkerArray markers;
    
    // Publish Arrow for the goal
    publishSingleGoalMarker(candidates[0].goal_point, markers, 0);
    // Publish Text Label
    publishTextMarker(candidates[0].goal_point.point, markers, 1, "CHECK REMOVED");
    // Publish Center for context
    publishCentroidMarker(clusters[candidates[0].cluster_id], markers, 2);

    goal_pub_->publish(markers);
  }

  // --- Helpers ---
  std::vector<geometry_msgs::msg::PointStamped> generateCircularGoals(const std::vector<PointXY>& cluster, const rclcpp::Time& stamp) {
      std::vector<geometry_msgs::msg::PointStamped> goals;
      if (cluster.empty()) return goals;

      // Centroid
      float cx = 0, cy = 0;
      for (auto& p : cluster) { cx += p.x; cy += p.y; }
      cx /= cluster.size(); cy /= cluster.size();

      double angle_step = 2.0 * M_PI / num_goal_points_;
      for (int i = 0; i < num_goal_points_; ++i) {
          double angle = i * angle_step;
          geometry_msgs::msg::PointStamped p;
          p.header.frame_id = map_frame_;
          p.header.stamp = stamp;
          p.point.x = cx + goal_dist_from_center_ * std::cos(angle);
          p.point.y = cy + goal_dist_from_center_ * std::sin(angle);
          p.point.z = 0.0;
          goals.push_back(p);
      }
      return goals;
  }

  void publishSingleGoalMarker(const geometry_msgs::msg::PointStamped& goal, visualization_msgs::msg::MarkerArray &markers, int id) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = goal.header.frame_id;
      m.header.stamp = this->now();
      m.ns = "removed_goal";
      m.id = id; 
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position = goal.point;
      m.pose.orientation.w = 1.0; 
      m.scale.x = 0.3; m.scale.y = 0.05; m.scale.z = 0.05;
      m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 1.0; // Purple
      m.lifetime = rclcpp::Duration::from_seconds(0.5); 
      markers.markers.push_back(m);
  }

  void publishCentroidMarker(const std::vector<PointXY>& cluster, visualization_msgs::msg::MarkerArray &markers, int id) {
      float cx = 0, cy = 0;
      for (auto& p : cluster) { cx += p.x; cy += p.y; }
      cx /= cluster.size(); cy /= cluster.size();

      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = this->now();
      m.ns = "removed_center";
      m.id = id; m.type = visualization_msgs::msg::Marker::SPHERE; 
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = cx; m.pose.position.y = cy;
      m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.2;
      m.color.r = 0.5; m.color.g = 0.0; m.color.b = 0.5; m.color.a = 0.8;
      m.lifetime = rclcpp::Duration::from_seconds(0.5); 
      markers.markers.push_back(m);
  }

  void publishTextMarker(const geometry_msgs::msg::Point& pos, visualization_msgs::msg::MarkerArray &markers, int id, const std::string& text) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = this->now();
      m.ns = "removed_text";
      m.id = id; m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position = pos; m.pose.position.z += 0.5;
      m.scale.z = 0.2; 
      m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0;
      m.text = text;
      m.lifetime = rclcpp::Duration::from_seconds(0.5); 
      markers.markers.push_back(m);
  }

  void publishClearMarkers() {
      visualization_msgs::msg::MarkerArray ma;
      visualization_msgs::msg::Marker m;
      m.action = visualization_msgs::msg::Marker::DELETEALL;
      ma.markers.push_back(m);
      goal_pub_->publish(ma);
  }

  // --- Clustering Logic (DBSCAN) ---
  std::vector<std::vector<PointXY>> extractClusters(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      std::vector<std::vector<PointXY>> clusters;
      std::vector<PointXY> points;
      sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
      for (; in_x != in_x.end(); ++in_x, ++in_y) points.push_back({*in_x, *in_y});

      std::vector<bool> visited(points.size(), false);
      for (size_t i = 0; i < points.size(); ++i) {
          if (visited[i]) continue;
          std::vector<PointXY> current_cluster;
          current_cluster.push_back(points[i]);
          visited[i] = true;

          for (size_t j = 0; j < current_cluster.size(); ++j) {
              PointXY p = current_cluster[j];
              for (size_t k = 0; k < points.size(); ++k) {
                  if (visited[k]) continue;
                  if (std::hypot(p.x - points[k].x, p.y - points[k].y) < cluster_dist_thresh_) {
                      visited[k] = true;
                      current_cluster.push_back(points[k]);
                  }
              }
          }
          if (current_cluster.size() >= (size_t)min_cluster_pts_) clusters.push_back(current_cluster);
      }
      return clusters;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RemovedObjectGoalSelector>());
  rclcpp::shutdown();
  return 0;
}