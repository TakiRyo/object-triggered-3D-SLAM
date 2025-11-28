#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

// --- Tracking Structure ---
struct TrackedCluster
{
  float min_x, max_x;
  float min_y, max_y;
  float cx, cy;
  float width, height;
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
  bool stable;
  float lock_radius; // Dynamic radius for matching
};

class ObjectClusterMarker : public rclcpp::Node
{
public:
  ObjectClusterMarker()
  : Node("object_goal_selector")
  {
    // --- Parameters ---
    this->declare_parameter("cluster_distance_threshold", 4.0); // Tight clustering
    this->declare_parameter("min_cluster_points", 8);          // Noise rejection
    this->declare_parameter("wall_thickness_threshold", 0.3);   // Reject thin walls
    this->declare_parameter("stability_time", 3.0);             // Time to become Green
    this->declare_parameter("lock_margin", 3.0);                // Extra buffer for lock radius
    this->declare_parameter("smoothing_factor", 1.0);           // Low-pass filter alpha

    cluster_distance_threshold_ = this->get_parameter("cluster_distance_threshold").as_double();
    min_cluster_points_         = this->get_parameter("min_cluster_points").as_int();
    wall_thickness_threshold_   = this->get_parameter("wall_thickness_threshold").as_double();
    stability_time_             = this->get_parameter("stability_time").as_double();
    lock_margin_                = this->get_parameter("lock_margin").as_double();
    smoothing_factor_           = this->get_parameter("smoothing_factor").as_double();

    // --- Subscribers & Publishers ---
    object_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/object_clusters", 10,
      std::bind(&ObjectClusterMarker::cloudCallback, this, std::placeholders::_1));

    candidate_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/candidate_clusters", 10);
    stable_pub_    = this->create_publisher<visualization_msgs::msg::MarkerArray>("/stable_clusters", 10);
    debug_pub_     = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug_lock_zones", 10);

    // --- Service for Mode Switching (Freeze/Unfreeze) ---
    mode_service_ = this->create_service<std_srvs::srv::SetBool>(
        "set_tracking_mode",
        std::bind(&ObjectClusterMarker::handleModeSwitch, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "✅ ObjectTracker Ready.");
    RCLCPP_INFO(this->get_logger(), "   - Dist: %.2f, Pts: %d, Wall: %.2f", cluster_distance_threshold_, min_cluster_points_, wall_thickness_threshold_);
    RCLCPP_INFO(this->get_logger(), "   - Stable: %.1fs, Margin: %.2fm, Smooth: %.2f", stability_time_, lock_margin_, smoothing_factor_);
  }

private:
  // --- State Variables ---
  bool tracking_enabled_ = true; // true = Active (Lidar Time), false = Frozen (Camera Time)
  std::vector<TrackedCluster> candidates_;      
  std::vector<TrackedCluster> stable_objects_;  

  // --- Service Callback ---
  void handleModeSwitch(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
      tracking_enabled_ = request->data;
      if (tracking_enabled_) {
          RCLCPP_INFO(this->get_logger(), "🔓 MODE: Active Tracking (Lidar Detection Time)");
          response->message = "Tracking Enabled";
      } else {
          RCLCPP_INFO(this->get_logger(), "🔒 MODE: FROZEN (Camera Detection Time)");
          response->message = "Tracking Frozen";
      }
      response->success = true;
  }

  // --- Helper: Calculate Lock Radius ---
  float calculateLockRadius(float w, float h) {
      // Radius = Half Diagonal + Margin
      float diagonal = std::sqrt(w*w + h*h);
      return (diagonal / 2.0f) + lock_margin_;
  }

  // --- Helper: Merge with Smoothing (Low-Pass Filter) ---
  void updateCluster(TrackedCluster &target, const TrackedCluster &source, rclcpp::Time now)
  {
    target.last_seen = now; 

    // Apply Smoothing to Position
    target.cx = (target.cx * (1.0f - smoothing_factor_)) + (source.cx * smoothing_factor_);
    target.cy = (target.cy * (1.0f - smoothing_factor_)) + (source.cy * smoothing_factor_);
    
    // Apply Smoothing to Size
    target.width  = (target.width * (1.0f - smoothing_factor_)) + (source.width * smoothing_factor_);
    target.height = (target.height * (1.0f - smoothing_factor_)) + (source.height * smoothing_factor_);
    
    // Update Lock Radius based on new size
    target.lock_radius = calculateLockRadius(target.width, target.height);
  }

  // --- Helper: Simple Absorb (Keep Alive) ---
  void absorbCluster(TrackedCluster &target, rclcpp::Time now) {
      target.last_seen = now; 
      // We do NOT update position here if we want strict "locking".
      // Or we can apply very aggressive smoothing (e.g. 0.01) if we want slight drift correction.
      // For now, let's keep it purely "Keep Alive" to minimize jitter.
  }

  // --- Main Lidar Callback ---
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    rclcpp::Time now = this->get_clock()->now();

    // =========================================================
    // 0. CHECK MODE: If Frozen, Skip Processing & Just Publish
    // =========================================================
    if (!tracking_enabled_) {
        // Republish existing markers so they don't disappear in RViz
        publishMarkers(now, msg->header.frame_id);
        return; 
    }

    // =========================================================
    // 1. Point Extraction
    // =========================================================
    std::vector<std::pair<float, float>> points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
      points.emplace_back(*iter_x, *iter_y);
    
    if (points.empty()) return;

    // =========================================================
    // 2. Clustering (Distance-Based)
    // =========================================================
    std::vector<std::vector<std::pair<float, float>>> clusters;
    std::vector<std::pair<float, float>> current;
    current.push_back(points.front());
    
    for (size_t i = 1; i < points.size(); ++i) {
      float dx = points[i].first - points[i-1].first;
      float dy = points[i].second - points[i-1].second;
      float dist = std::sqrt(dx*dx + dy*dy);
      
      if (dist > cluster_distance_threshold_) {
        if (current.size() >= (size_t)min_cluster_points_) 
            clusters.push_back(current);
        current.clear();
      }
      current.push_back(points[i]);
    }
    // Add last cluster
    if (current.size() >= (size_t)min_cluster_points_) 
        clusters.push_back(current);

    // =========================================================
    // 3. Raw Cluster Generation & Wall Filtering
    // =========================================================
    std::vector<TrackedCluster> raw_observations;
    for (const auto &cluster : clusters) {
        float min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
        for (const auto &p : cluster) {
            min_x = std::min(min_x, p.first); max_x = std::max(max_x, p.first);
            min_y = std::min(min_y, p.second); max_y = std::max(max_y, p.second);
        }
        float width = max_x - min_x; 
        float height = max_y - min_y;

        // Filter Thin Walls
        if (std::min(width, height) < wall_thickness_threshold_) continue;

        TrackedCluster raw;
        raw.cx = (min_x + max_x)/2.0f; 
        raw.cy = (min_y + max_y)/2.0f;
        raw.width = width; 
        raw.height = height;
        raw.lock_radius = calculateLockRadius(width, height);
        raw.first_seen = now; 
        raw.last_seen = now; 
        raw.stable = false;
        
        raw_observations.push_back(raw);
    }

    // =========================================================
    // 4. Matching Logic (Priority: Stable > Candidate > New)
    // =========================================================
    for (auto &raw : raw_observations) {
        bool matched = false;

        // A. Check against STABLE (Green)
        for (auto &stable : stable_objects_) {
            float dist = std::hypot(raw.cx - stable.cx, raw.cy - stable.cy);
            if (dist < stable.lock_radius) {
                // If inside lock zone, absorb it (update last_seen) but potentially don't move it
                // Using updateCluster here applies smoothing. 
                // Using absorbCluster keeps it strictly fixed.
                updateCluster(stable, raw, now); 
                matched = true; 
                break;
            }
        }
        if (matched) continue;

        // B. Check against CANDIDATES (Yellow)
        for (auto &cand : candidates_) {
             float dist = std::hypot(raw.cx - cand.cx, raw.cy - cand.cy);
             // Use same radius logic for candidates
             if (dist < cand.lock_radius) {
                updateCluster(cand, raw, now); 
                matched = true; 
                break;
            }
        }
        if (!matched) {
            candidates_.push_back(raw);
        }
    }

    // =========================================================
    // 5. Maintenance (Promote & Delete)
    // =========================================================
    auto it = candidates_.begin();
    while (it != candidates_.end()) {
        double age = (now - it->first_seen).seconds();
        double unseen = (now - it->last_seen).seconds();

        // Promote to Stable
        if (age > stability_time_) {
            it->stable = true;
            stable_objects_.push_back(*it);
            it = candidates_.erase(it);
            RCLCPP_INFO(this->get_logger(), "🔒 Object Locked (Green).");
            continue;
        }
        // Delete Noise
        if (unseen > 0.5) {
            it = candidates_.erase(it);
        } else {
            ++it;
        }
    }

    // =========================================================
    // 6. Publish Markers
    // =========================================================
    publishMarkers(now, msg->header.frame_id);
  }

  // --- Visualization Publisher ---
  void publishMarkers(rclcpp::Time now, std::string frame_id) {
      visualization_msgs::msg::MarkerArray candidate_array, stable_array, debug_array;
      int id = 0;

      // Visual Feedback for Mode:
      // Active = Green (0, 1, 0)
      // Frozen = Blue (0, 0, 1)
      float r_val = 0.0f;
      float g_val = tracking_enabled_ ? 1.0f : 0.0f;
      float b_val = tracking_enabled_ ? 0.0f : 1.0f;

      // 1. Stable Objects
      for (const auto &c : stable_objects_) {
          visualization_msgs::msg::Marker m;
          m.header.frame_id = frame_id; m.header.stamp = now;
          m.ns = "stable"; m.id = id++; m.type = visualization_msgs::msg::Marker::CUBE;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.pose.position.x = c.cx; m.pose.position.y = c.cy; m.pose.position.z = 0.2;
          m.scale.x = c.width; m.scale.y = c.height; m.scale.z = 0.4;
          m.color.r = r_val; m.color.g = g_val; m.color.b = b_val; m.color.a = 0.8;
          stable_array.markers.push_back(m);

          // Lock Zone (Red Circle)
          visualization_msgs::msg::Marker zone;
          zone.header.frame_id = frame_id; zone.header.stamp = now;
          zone.ns = "lock_zone"; zone.id = id++; zone.type = visualization_msgs::msg::Marker::CYLINDER;
          zone.action = visualization_msgs::msg::Marker::ADD;
          zone.pose.position.x = c.cx; zone.pose.position.y = c.cy; zone.pose.position.z = 0.0;
          zone.scale.x = c.lock_radius * 2.0; 
          zone.scale.y = c.lock_radius * 2.0; 
          zone.scale.z = 0.05;
          zone.color.r = 1.0; zone.color.g = 0.0; zone.color.b = 0.0; zone.color.a = 0.1;
          debug_array.markers.push_back(zone);
      }

      // 2. Candidates
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
  }

  // --- Members ---
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr object_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stable_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service_;

  double cluster_distance_threshold_;
  int min_cluster_points_;
  double wall_thickness_threshold_;
  double stability_time_;
  double lock_margin_;
  double smoothing_factor_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectClusterMarker>());
  rclcpp::shutdown();
  return 0;
}