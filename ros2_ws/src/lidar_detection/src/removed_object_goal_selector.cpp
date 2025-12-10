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
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

// Simple point structure for clustering
struct PointXY { float x, y; };

// Struct to hold a candidate goal and its distance to the robot
struct CandidateGoal {
    geometry_msgs::msg::PointStamped goal_point;
    double distance;
    int cluster_id;
};

class RemovedObjectGoalSelector : public rclcpp::Node
{
public:
  RemovedObjectGoalSelector() : Node("removed_object_goal_selector")
  {
    // --- TF Setup ---
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- Parameters ---
    this->declare_parameter("cluster_distance_threshold", 0.4);
    this->declare_parameter("min_cluster_points", 8);          
    this->declare_parameter("goal_distance_from_center", 0.6); // Distance for circular points
    this->declare_parameter("num_goal_points_per_cluster", 8); // Number of points in the circle
    this->declare_parameter("robot_base_frame", "base_link"); // Robot's frame (from the TF tree)
    this->declare_parameter("map_frame", "map");             // Global map frame

    cluster_dist_thresh_ = this->get_parameter("cluster_distance_threshold").as_double();
    min_cluster_pts_     = this->get_parameter("min_cluster_points").as_int();
    goal_dist_from_center_ = this->get_parameter("goal_distance_from_center").as_double();
    num_goal_points_     = this->get_parameter("num_goal_points_per_cluster").as_int();
    robot_base_frame_    = this->get_parameter("robot_base_frame").as_string();
    map_frame_           = this->get_parameter("map_frame").as_string();

    // --- Subscribers & Publisher ---
    removed_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/removed_objects", 10,
      std::bind(&RemovedObjectGoalSelector::removedCallback, this, std::placeholders::_1));

    // Publishes the SINGLE chosen goal for removed objects
    goal_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/removed_object_visiting_points", 10);

    RCLCPP_INFO(this->get_logger(), "Removed Object Goal Selector Ready. Publishing single nearest goal.");
  }

private:
  // Components
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr removed_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_pub_;

  // Parameters
  double cluster_dist_thresh_;
  int min_cluster_pts_;
  double goal_dist_from_center_;
  int num_goal_points_;
  std::string robot_base_frame_;
  std::string map_frame_;


  void removedCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1. Clear Markers if no objects
    if (msg->data.empty()) {
        publishClearMarkers();
        return;
    }

    // Get Robot Pose
    geometry_msgs::msg::TransformStamped robot_transform;
    try {
        robot_transform = tf_buffer_->lookupTransform(
            map_frame_, robot_base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
            robot_base_frame_.c_str(), map_frame_.c_str(), ex.what());
        publishClearMarkers();
        return;
    }
    double robot_x = robot_transform.transform.translation.x;
    double robot_y = robot_transform.transform.translation.y;


    // 2. Extract and Process Clusters
    auto clusters = extractClusters(msg);
    if (clusters.empty()) {
        publishClearMarkers();
        return;
    }

    std::vector<CandidateGoal> all_candidate_goals;

    for (size_t i = 0; i < clusters.size(); ++i) {
        // 3. Generate Circular Goals for each cluster
        auto points = generateCircularGoals(clusters[i], i, msg->header.stamp);
        
        // 4. Calculate Distance and Collect Candidates
        for (auto& p : points) {
            double dist = std::hypot(p.point.x - robot_x, p.point.y - robot_y);
            all_candidate_goals.push_back({p, dist, (int)i});
        }
    }

    // 5. Select the Single Nearest Goal
    if (all_candidate_goals.empty()) {
        publishClearMarkers();
        return;
    }

    auto nearest_goal_it = std::min_element(all_candidate_goals.begin(), all_candidate_goals.end(),
        [](const CandidateGoal& a, const CandidateGoal& b) {
            return a.distance < b.distance;
        });

    // 6. Publish the Single Nearest Goal as a Marker
    visualization_msgs::msg::MarkerArray markers;
    
    // Publish the chosen goal (Marker ID 0)
    publishSingleGoalMarker(nearest_goal_it->goal_point, markers, 0);

    // Publish the original cluster centroid for visualization (Marker ID 1)
    publishCentroidMarker(clusters[nearest_goal_it->cluster_id], markers, 1);
    
    // Add a text marker to label the chosen goal (Marker ID 2)
    publishTextMarker(nearest_goal_it->goal_point.point, markers, 2, "NEAREST REMOVED OBJECT CHECK POINT");

    goal_pub_->publish(markers);
  }

  // --- Helper Functions ---

  void publishClearMarkers() {
      visualization_msgs::msg::MarkerArray clear_markers;
      visualization_msgs::msg::Marker clear_marker;
      clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      clear_markers.markers.push_back(clear_marker);
      goal_pub_->publish(clear_markers);
  }


  std::vector<geometry_msgs::msg::PointStamped> generateCircularGoals(const std::vector<PointXY>& cluster, int obj_id, const rclcpp::Time& stamp) {
      std::vector<geometry_msgs::msg::PointStamped> goals;
      if (cluster.empty()) return goals;

      // Compute Center (Centroid)
      float cx = 0, cy = 0;
      for (auto& p : cluster) { cx += p.x; cy += p.y; }
      cx /= cluster.size(); cy /= cluster.size();

      double angle_step = 2.0 * M_PI / num_goal_points_;

      for (int i = 0; i < num_goal_points_; ++i) {
          double angle = i * angle_step;

          geometry_msgs::msg::PointStamped p;
          p.header.frame_id = map_frame_;
          p.header.stamp = stamp;
          
          // Calculate goal position
          p.point.x = cx + goal_dist_from_center_ * std::cos(angle);
          p.point.y = cy + goal_dist_from_center_ * std::sin(angle);
          p.point.z = 0.0; // Assuming 2D planning

          goals.push_back(p);
      }
      return goals;
  }
  
  void publishSingleGoalMarker(const geometry_msgs::msg::PointStamped& goal, visualization_msgs::msg::MarkerArray &markers, int id) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = goal.header.frame_id;
      m.header.stamp = goal.header.stamp;
      m.ns = "single_removed_goal";
      m.id = id; 
      m.type = visualization_msgs::msg::Marker::ARROW; 
      m.action = visualization_msgs::msg::Marker::ADD;
      
      // Goal Position (P1)
      m.pose.position = goal.point;
      
      // To make it an Arrow, we need a better orientation calculation. 
      // For now, we'll just use a SPHERE to denote the point.
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.pose.orientation.w = 1.0; 

      m.scale.x = 0.4; m.scale.y = 0.4; m.scale.z = 0.4;
      m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 1.0; // Bright Purple 
      
      // Ensure the marker is deleted if not updated
      m.lifetime = rclcpp::Duration::from_seconds(0.5); 
      
      markers.markers.push_back(m);
  }

  void publishCentroidMarker(const std::vector<PointXY>& cluster, visualization_msgs::msg::MarkerArray &markers, int id) {
      // Compute Center (Centroid)
      float cx = 0, cy = 0;
      for (auto& p : cluster) { cx += p.x; cy += p.y; }
      cx /= cluster.size(); cy /= cluster.size();

      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = this->now();
      m.ns = "removed_centroid";
      m.id = id; 
      m.type = visualization_msgs::msg::Marker::SPHERE; 
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = cx;
      m.pose.position.y = cy;
      m.pose.orientation.w = 1.0;

      m.scale.x = 0.1; m.scale.y = 0.1; m.scale.z = 0.1;
      m.color.r = 0.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0; // Black for centroid
      
      // Ensure the marker is deleted if not updated
      m.lifetime = rclcpp::Duration::from_seconds(0.5); 

      markers.markers.push_back(m);
  }

  void publishTextMarker(const geometry_msgs::msg::Point& pos, visualization_msgs::msg::MarkerArray &markers, int id, const std::string& text) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = this->now();
      m.ns = "removed_goal_text";
      m.id = id; 
      m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = pos.x;
      m.pose.position.y = pos.y + 0.5; // Offset text slightly above the goal
      m.pose.orientation.w = 1.0;
      m.text = text;

      m.scale.z = 0.2; // Text height
      m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0; // White text
      
      m.lifetime = rclcpp::Duration::from_seconds(0.5); 

      markers.markers.push_back(m);
  }


  // --- Helper: Cluster Points (DBSCAN-like) ---
  std::vector<std::vector<PointXY>> extractClusters(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      // Implementation is the same as in the previous example
      // (Using the same logic ensures consistency for both removed and added objects)
      // ... [The clustering logic from the previous reply goes here] ...
      
      std::vector<std::vector<PointXY>> clusters;
      std::vector<PointXY> points;
      
      sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");

      for (; in_x != in_x.end(); ++in_x, ++in_y) {
          points.push_back({*in_x, *in_y});
      }

      std::vector<bool> visited(points.size(), false);
      for (size_t i = 0; i < points.size(); ++i) {
          if (visited[i]) continue;
          
          std::vector<PointXY> current_cluster;
          current_cluster.push_back(points[i]);
          visited[i] = true;

          // Simple Breadth-First-Search like expansion using current_cluster size
          for (size_t j = 0; j < current_cluster.size(); ++j) {
              PointXY p = current_cluster[j];
              for (size_t k = 0; k < points.size(); ++k) {
                  if (visited[k]) continue;
                  float dist = std::hypot(p.x - points[k].x, p.y - points[k].y);
                  if (dist < cluster_dist_thresh_) {
                      visited[k] = true;
                      current_cluster.push_back(points[k]);
                  }
              }
          }
          if (current_cluster.size() >= (size_t)min_cluster_pts_) {
              clusters.push_back(current_cluster);
          }
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