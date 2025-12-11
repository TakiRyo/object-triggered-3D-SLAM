/*
 * Node Name: ObjectClusterMarker
 * Role: Tracker & Goal Generator
 * Update: 
 * - FIXED: Added missing declaration for 'min_cluster_points_'.
 * - Feature: Uniform Distance Scanning (Equal spacing for small & big objects).
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

struct TrackedCluster
{
  float min_x, max_x;
  float min_y, max_y;
  float cx, cy;
  float width, height;
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
  bool stable;
  float lock_radius; 
};

class ObjectClusterMarker : public rclcpp::Node
{
public:
  ObjectClusterMarker()
  : Node("object_goal_selector")
  {
    // --- Parameters ---
    this->declare_parameter("cluster_distance_threshold", 0.4);
    this->declare_parameter("min_cluster_points", 10);          
    this->declare_parameter("wall_thickness_threshold", 0.2);   
    this->declare_parameter("stability_time", 3.0);             
    this->declare_parameter("lock_margin", 0.5);                
    this->declare_parameter("smoothing_factor", 1.0); 
    this->declare_parameter("visiting_point_buffer", 0.2); 
    this->declare_parameter("scan_step_threshold", 1.2); 

    // Uniform Spacing Parameters
    this->declare_parameter("scan_point_interval", 1.0); 
    this->declare_parameter("scan_point_interval_removed", 5.0); 
    this->declare_parameter("min_scan_points", 4); 

    this->declare_parameter("global_frame", "map"); 

    cluster_distance_threshold_ = this->get_parameter("cluster_distance_threshold").as_double();
    min_cluster_points_         = this->get_parameter("min_cluster_points").as_int();
    wall_thickness_threshold_   = this->get_parameter("wall_thickness_threshold").as_double();
    stability_time_             = this->get_parameter("stability_time").as_double();
    lock_margin_                = this->get_parameter("lock_margin").as_double();
    smoothing_factor_           = this->get_parameter("smoothing_factor").as_double();
    visiting_point_buffer_      = this->get_parameter("visiting_point_buffer").as_double();
    
    scan_step_threshold_        = this->get_parameter("scan_step_threshold").as_double();
    
    scan_point_interval_        = this->get_parameter("scan_point_interval").as_double();
    scan_point_interval_removed_ = this->get_parameter("scan_point_interval_removed").as_double();
    min_scan_points_            = this->get_parameter("min_scan_points").as_int();
    
    global_frame_               = this->get_parameter("global_frame").as_string();

    // --- Subscribers & Publishers ---
    object_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/object_clusters", 10,
      std::bind(&ObjectClusterMarker::cloudCallback, this, std::placeholders::_1));

    candidate_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/candidate_clusters", 10);
    stable_pub_    = this->create_publisher<visualization_msgs::msg::MarkerArray>("/stable_clusters", 10);
    debug_pub_     = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug_lock_zones", 10);
    goal_pub_      = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_visiting_points", 10);

    mode_service_ = this->create_service<std_srvs::srv::SetBool>(
        "set_tracking_mode",
        std::bind(&ObjectClusterMarker::handleModeSwitch, this, std::placeholders::_1, std::placeholders::_2));

    // TF2 Setup
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // RCLCPP_INFO(this->get_logger(), "✅ ObjectTracker Ready. Mode: Uniform Distance Scan (%.2fm)", scan_point_interval_);

    // Additional Subscriber & Publishers for Removed Clusters
    removed_object_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/removed_objects", 10,
      std::bind(&ObjectClusterMarker::removedCloudCallback, this, std::placeholders::_1));

    removed_stable_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/removed_stable_clusters", 10);
    removed_goal_pub_   = this->create_publisher<visualization_msgs::msg::MarkerArray>("/removed_object_visiting_points", 10);
  }

private:
  bool tracking_enabled_ = true; 
  std::vector<TrackedCluster> candidates_;      
  std::vector<TrackedCluster> stable_objects_;
  std::vector<TrackedCluster> removed_candidates_;
  std::vector<TrackedCluster> removed_stable_objects_;
  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  double scan_point_interval_;
  double scan_point_interval_removed_;
  int min_scan_points_;
  int min_cluster_points_; // <--- Re-added this missing declaration!

  double cluster_distance_threshold_, wall_thickness_threshold_, stability_time_, lock_margin_, smoothing_factor_, visiting_point_buffer_;
  double scan_step_threshold_; 
  
  void handleModeSwitch(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
      tracking_enabled_ = request->data;
      if (tracking_enabled_) {
          RCLCPP_INFO(this->get_logger(), "🔓 MODE: Active (Lidar Time)");
          response->message = "Tracking Enabled";
      } else {
          RCLCPP_INFO(this->get_logger(), "🔒 MODE: FROZEN (Camera Time)");
          response->message = "Tracking Frozen";
      }
      response->success = true;
  }

  float calculateLockRadius(float w, float h) {
      float diagonal = std::sqrt(w*w + h*h);
      return (diagonal / 2.0f) + lock_margin_;
  }

  void updateCluster(TrackedCluster &target, const TrackedCluster &source, rclcpp::Time now)
  {
    target.last_seen = now; 
    target.cx = (target.cx * (1.0f - smoothing_factor_)) + (source.cx * smoothing_factor_);
    target.cy = (target.cy * (1.0f - smoothing_factor_)) + (source.cy * smoothing_factor_);
    target.width  = (target.width * (1.0f - smoothing_factor_)) + (source.width * smoothing_factor_);
    target.height = (target.height * (1.0f - smoothing_factor_)) + (source.height * smoothing_factor_);
    target.lock_radius = calculateLockRadius(target.width, target.height);
  }

  void absorbToStable(TrackedCluster &target, rclcpp::Time now) {
      target.last_seen = now; 
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    rclcpp::Time now = this->get_clock()->now();

    if (!tracking_enabled_) {
        publishMarkers(now, global_frame_);
        return; 
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        global_frame_, msg->header.frame_id, 
        tf2::TimePointZero); 
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
      return;
    }

    std::vector<std::pair<float, float>> points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) points.emplace_back(*iter_x, *iter_y);
    if (points.empty()) return;

    std::vector<std::vector<std::pair<float, float>>> clusters;
    std::vector<std::pair<float, float>> current;
    current.push_back(points.front());
    for (size_t i = 1; i < points.size(); ++i) {
      float dx = points[i].first - points[i-1].first;
      float dy = points[i].second - points[i-1].second;
      if (std::sqrt(dx*dx + dy*dy) > cluster_distance_threshold_) {
        if (current.size() >= (size_t)min_cluster_points_) clusters.push_back(current);
        current.clear();
      }
      current.push_back(points[i]);
    }
    if (current.size() >= (size_t)min_cluster_points_) clusters.push_back(current);

    std::vector<TrackedCluster> raw_observations;
    for (const auto &cluster : clusters) {
        float min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
        for (const auto &p : cluster) {
            min_x = std::min(min_x, p.first); max_x = std::max(max_x, p.first);
            min_y = std::min(min_y, p.second); max_y = std::max(max_y, p.second);
        }
        float w = max_x - min_x; float h = max_y - min_y;
        if (std::min(w, h) < wall_thickness_threshold_) continue;

        float local_cx = (min_x + max_x)/2.0f;
        float local_cy = (min_y + max_y)/2.0f;
        geometry_msgs::msg::PointStamped local_pt, map_pt;
        local_pt.point.x = local_cx; local_pt.point.y = local_cy; local_pt.point.z = 0.0;
        tf2::doTransform(local_pt, map_pt, transform_stamped);

        TrackedCluster raw;
        raw.cx = map_pt.point.x; 
        raw.cy = map_pt.point.y;
        raw.width = w; raw.height = h;
        raw.lock_radius = calculateLockRadius(w, h);
        raw.first_seen = now; raw.last_seen = now; raw.stable = false;
        raw_observations.push_back(raw);
    }

    for (auto &raw : raw_observations) {
        bool matched = false;
        for (auto &stable : stable_objects_) {
            if (std::hypot(raw.cx - stable.cx, raw.cy - stable.cy) < stable.lock_radius) {
                absorbToStable(stable, now); matched = true; break;
            }
        }
        if (matched) continue;
        for (auto &cand : candidates_) {
             if (std::hypot(raw.cx - cand.cx, raw.cy - cand.cy) < cand.lock_radius) {
                updateCluster(cand, raw, now); matched = true; break;
            }
        }
        if (!matched) candidates_.push_back(raw);
    }

    auto it = candidates_.begin();
    while (it != candidates_.end()) {
        double age = (now - it->first_seen).seconds();
        double unseen = (now - it->last_seen).seconds();
        if (age > stability_time_) {
            it->stable = true;
            stable_objects_.push_back(*it);
            it = candidates_.erase(it);
        } else if (unseen > 0.5) {
            it = candidates_.erase(it);
        } else {
            ++it;
        }
    }

    publishMarkers(now, global_frame_);
  }

  void publishMarkers(rclcpp::Time now, std::string frame_id) {
      visualization_msgs::msg::MarkerArray candidate_array, stable_array, debug_array, goals_array;
      int id = 0;
      float r=0.0f, g=tracking_enabled_?1.0f:0.0f, b=tracking_enabled_?0.0f:1.0f;

      for (const auto &c : stable_objects_) {
          visualization_msgs::msg::Marker m;
          m.header.frame_id = frame_id; m.header.stamp = now;
          m.ns = "stable"; m.id = id++; m.type = visualization_msgs::msg::Marker::CUBE;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.pose.position.x = c.cx; m.pose.position.y = c.cy; m.pose.position.z = 0.2;
          m.scale.x = c.width; m.scale.y = c.height; m.scale.z = 0.4;
          m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.8;
          stable_array.markers.push_back(m);

          visualization_msgs::msg::Marker zone;
          zone.header.frame_id = frame_id; zone.header.stamp = now;
          zone.ns = "lock_zone"; zone.id = id++; zone.type = visualization_msgs::msg::Marker::CYLINDER;
          zone.action = visualization_msgs::msg::Marker::ADD;
          zone.pose.position.x = c.cx; zone.pose.position.y = c.cy; zone.pose.position.z = 0.0;
          zone.scale.x = c.lock_radius * 2.0; zone.scale.y = c.lock_radius * 2.0; zone.scale.z = 0.05;
          zone.color.r = 1.0; zone.color.a = 0.1;
          debug_array.markers.push_back(zone);

          // ★ UNIFORM DISTANCE VISITING POINTS ★
          float vp_radius = c.lock_radius + visiting_point_buffer_;
          int obj_id_index = &c - &stable_objects_[0]; 

          double circumference = 2.0 * M_PI * vp_radius;
          int num_points = std::max(min_scan_points_, (int)std::round(circumference / scan_point_interval_));
          double angle_step = (2.0 * M_PI) / (double)num_points;

          for (int i = 0; i < num_points; i++) {
              visualization_msgs::msg::Marker p;
              p.header.frame_id = frame_id; p.header.stamp = now;
              p.ns = "visiting_points";
              p.id = (obj_id_index * 100) + i; 

              p.type = visualization_msgs::msg::Marker::ARROW;
              p.action = visualization_msgs::msg::Marker::ADD;

              double angle = angle_step * i;

              p.pose.position.x = c.cx + vp_radius * std::cos(angle);
              p.pose.position.y = c.cy + vp_radius * std::sin(angle);
              p.pose.position.z = 0.0; 

              double yaw = angle + M_PI; 

              p.pose.orientation.w = std::cos(yaw * 0.5);
              p.pose.orientation.z = std::sin(yaw * 0.5);
              p.pose.orientation.x = 0.0;
              p.pose.orientation.y = 0.0;

              p.scale.x = 0.3; p.scale.y = 0.05; p.scale.z = 0.05;
              p.color.r = 0.0; p.color.g = 1.0; p.color.b = 1.0; p.color.a = 0.9;
              goals_array.markers.push_back(p);
          }
      }

      for (const auto &c : candidates_) {
          visualization_msgs::msg::Marker m;
          m.header.frame_id = frame_id; m.header.stamp = now;
          m.ns = "candidate"; m.id = id++; m.type = visualization_msgs::msg::Marker::CUBE;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.pose.position.x = c.cx; m.pose.position.y = c.cy; m.pose.position.z = 0.2;
          m.scale.x = c.width; m.scale.y = c.height; m.scale.z = 0.4;
          m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.6;
          candidate_array.markers.push_back(m);
      }

      stable_pub_->publish(stable_array);
      candidate_pub_->publish(candidate_array);
      debug_pub_->publish(debug_array);
      goal_pub_->publish(goals_array); 
  }

  void removedCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    rclcpp::Time now = this->get_clock()->now();

    if (!tracking_enabled_) {
        publishRemovedMarkers(now, global_frame_);
        return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        global_frame_, msg->header.frame_id, 
        tf2::TimePointZero); 
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
      return;
    }

    std::vector<std::pair<float, float>> points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) points.emplace_back(*iter_x, *iter_y);
    if (points.empty()) return;

    std::vector<std::vector<std::pair<float, float>>> clusters;
    std::vector<std::pair<float, float>> current;
    current.push_back(points.front());
    for (size_t i = 1; i < points.size(); ++i) {
      float dx = points[i].first - points[i-1].first;
      float dy = points[i].second - points[i-1].second;
      if (std::sqrt(dx*dx + dy*dy) > cluster_distance_threshold_) {
        if (current.size() >= (size_t)min_cluster_points_) clusters.push_back(current);
        current.clear();
      }
      current.push_back(points[i]);
    }
    if (current.size() >= (size_t)min_cluster_points_) clusters.push_back(current);

    std::vector<TrackedCluster> raw_observations;
    for (const auto &cluster : clusters) {
        float min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
        for (const auto &p : cluster) {
            min_x = std::min(min_x, p.first); max_x = std::max(max_x, p.first);
            min_y = std::min(min_y, p.second); max_y = std::max(max_y, p.second);
        }
        float w = max_x - min_x; float h = max_y - min_y;
        if (std::min(w, h) < wall_thickness_threshold_) continue;

        float local_cx = (min_x + max_x)/2.0f;
        float local_cy = (min_y + max_y)/2.0f;
        geometry_msgs::msg::PointStamped local_pt, map_pt;
        local_pt.point.x = local_cx; local_pt.point.y = local_cy; local_pt.point.z = 0.0;
        tf2::doTransform(local_pt, map_pt, transform_stamped);

        TrackedCluster raw;
        raw.cx = map_pt.point.x; 
        raw.cy = map_pt.point.y;
        raw.width = w; raw.height = h;
        raw.lock_radius = calculateLockRadius(w, h);
        raw.first_seen = now; raw.last_seen = now; raw.stable = false;
        raw_observations.push_back(raw);
    }

    for (auto &raw : raw_observations) {
        bool matched = false;
        for (auto &stable : removed_stable_objects_) {
            if (std::hypot(raw.cx - stable.cx, raw.cy - stable.cy) < stable.lock_radius) {
                absorbToStable(stable, now); matched = true; break;
            }
        }
        if (matched) continue;
        for (auto &cand : removed_candidates_) {
             if (std::hypot(raw.cx - cand.cx, raw.cy - cand.cy) < cand.lock_radius) {
                updateCluster(cand, raw, now); matched = true; break;
            }
        }
        if (!matched) removed_candidates_.push_back(raw);
    }

    auto it = removed_candidates_.begin();
    while (it != removed_candidates_.end()) {
        double age = (now - it->first_seen).seconds();
        double unseen = (now - it->last_seen).seconds();
        if (age > stability_time_) {
            it->stable = true;
            removed_stable_objects_.push_back(*it);
            it = removed_candidates_.erase(it);
        } else if (unseen > 0.5) {
            it = removed_candidates_.erase(it);
        } else {
            ++it;
        }
    }

    publishRemovedMarkers(now, global_frame_);
  }

  void publishRemovedMarkers(rclcpp::Time now, std::string frame_id) {
      visualization_msgs::msg::MarkerArray stable_array, goals_array;
      int id = 0;

      for (const auto &c : removed_stable_objects_) {
          visualization_msgs::msg::Marker m;
          m.header.frame_id = frame_id; m.header.stamp = now;
          m.ns = "removed_stable"; m.id = id++; m.type = visualization_msgs::msg::Marker::CUBE;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.pose.position.x = c.cx; m.pose.position.y = c.cy; m.pose.position.z = 0.2;
          m.scale.x = c.width; m.scale.y = c.height; m.scale.z = 0.4;
          m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.8;
          stable_array.markers.push_back(m);

          // Visiting Points for Removed Clusters
          float vp_radius = c.lock_radius + visiting_point_buffer_;
          int obj_id_index = &c - &removed_stable_objects_[0]; 

          double scan_point_interval_removed_ = 5.0; // Fixed interval for removed clusters
          double circumference = 2.0 * M_PI * vp_radius;
          int num_points = std::max(min_scan_points_, (int)std::round(circumference / scan_point_interval_removed_));
          double angle_step = (2.0 * M_PI) / (double)num_points;

          for (int i = 0; i < num_points; i++) {
              visualization_msgs::msg::Marker p;
              p.header.frame_id = frame_id; p.header.stamp = now;
              p.ns = "removed_visiting_points";
              p.id = (obj_id_index * 100) + i; 

              p.type = visualization_msgs::msg::Marker::ARROW;
              p.action = visualization_msgs::msg::Marker::ADD;

              double angle = angle_step * i;

              p.pose.position.x = c.cx + vp_radius * std::cos(angle);
              p.pose.position.y = c.cy + vp_radius * std::sin(angle);
              p.pose.position.z = 0.0; 

              double yaw = angle + M_PI; 

              p.pose.orientation.w = std::cos(yaw * 0.5);
              p.pose.orientation.z = std::sin(yaw * 0.5);
              p.pose.orientation.x = 0.0;
              p.pose.orientation.y = 0.0;

              p.scale.x = 0.3; p.scale.y = 0.05; p.scale.z = 0.05;
              p.color.r = 1.0; p.color.g = 0.0; p.color.b = 0.0; p.color.a = 0.9;
              goals_array.markers.push_back(p);
          }
      }

      removed_stable_pub_->publish(stable_array);
      removed_goal_pub_->publish(goals_array); 
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr object_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_pub_, stable_pub_, debug_pub_, goal_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr removed_object_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr removed_stable_pub_, removed_goal_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectClusterMarker>());
  rclcpp::shutdown();
  return 0;
}