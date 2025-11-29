/**
 * -----------------------------------------------------------------------
 * Node Name: ObjectClusterMarker
 * -----------------------------------------------------------------------
 * Purpose:
 * 1. Tracks distinct objects over time (Temporal Persistence).
 * 2. Filters noise by requiring objects to be stable for `stability_time`.
 * 3. Generates "Visiting Points" (Goals) around the object.
 * 4. Freezes the map state via service for navigation tasks.
 *
 * Input:
 * /object_clusters (sensor_msgs::msg::PointCloud2)
 * - A single cloud containing points classified as "Objects" by the previous node.
 *
 * Output:
 * /candidate_clusters   (MarkerArray) -> YELLOW Boxes (Unstable/New detections)
 * /stable_clusters      (MarkerArray) -> GREEN Boxes  (Confirmed objects)
 * /debug_lock_zones     (MarkerArray) -> RED Cylinders (The "Keep-Out" or "Lock" zone)
 * /object_visiting_points (MarkerArray) -> CYAN Arrows (Navigation Goals)
 *
 * Logic Flow:
 * 1. Re-Clustering:
 * Incoming points are grouped into individual clusters using Euclidean distance.
 * 2. Data Association:
 * - New clusters are matched to existing 'Stable' objects first.
 * - If no match, they are matched to 'Candidate' objects.
 * - If still no match, a new 'Candidate' is created.
 * 3. State Machine:
 * - Candidate -> Stable: If tracked consistently for > `stability_time`.
 * - Candidate -> Deleted: If not seen for > 0.5 seconds.
 * 4. Goal Generation (The "Visiting Points"):
 * - Creates 4 points (North, South, East, West) around the object's lock zone.
 * - **Orientation Calculation**:
 * The orientation (Yaw) is calculated so the arrow points **FROM** the visiting point
 * **TO** the center of the object.
 * Formula: yaw = atan2(object_cy - point_y, object_cx - point_x)
 *
 * Services:
 * /set_tracking_mode (std_srvs::SetBool)
 * - True: Updates positions based on LiDAR (Live Tracking).
 * - False: Freezes markers in place (allows robot to navigate to them without
 * the goal jumping around due to sensor noise).
 *
 * Adjustable Parameters:
 * stability_time        : Seconds a cluster must exist to become "Stable".
 * lock_margin           : Extra padding added to object radius for the lock zone.
 * visiting_point_buffer : Distance from the lock zone to the goal points.
 * smoothing_factor      : 0.0 to 1.0. Higher = faster updates, Lower = smoother motion.
 * -----------------------------------------------------------------------
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
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
    this->declare_parameter("cluster_distance_threshold", 0.4);
    this->declare_parameter("min_cluster_points", 8);          
    this->declare_parameter("wall_thickness_threshold", 0.3);   
    this->declare_parameter("stability_time", 3.0);             
    this->declare_parameter("lock_margin", 2.0);                
    this->declare_parameter("smoothing_factor", 1.0); 
    this->declare_parameter("visiting_point_buffer", 0.1); 

    cluster_distance_threshold_ = this->get_parameter("cluster_distance_threshold").as_double();
    min_cluster_points_         = this->get_parameter("min_cluster_points").as_int();
    wall_thickness_threshold_   = this->get_parameter("wall_thickness_threshold").as_double();
    stability_time_             = this->get_parameter("stability_time").as_double();
    lock_margin_                = this->get_parameter("lock_margin").as_double();
    smoothing_factor_           = this->get_parameter("smoothing_factor").as_double();
    visiting_point_buffer_      = this->get_parameter("visiting_point_buffer").as_double();

    // Input and Output
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

    RCLCPP_INFO(this->get_logger(), "✅ ObjectTracker Ready.");
  }

private:
  bool tracking_enabled_ = true; 
  std::vector<TrackedCluster> candidates_;      
  std::vector<TrackedCluster> stable_objects_;  

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
        publishMarkers(now, msg->header.frame_id);
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

        TrackedCluster raw;
        raw.cx = (min_x + max_x)/2.0f; raw.cy = (min_y + max_y)/2.0f;
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
            RCLCPP_INFO(this->get_logger(), "🔒 Object Locked (Green).");
        } else if (unseen > 0.5) {
            it = candidates_.erase(it);
        } else {
            ++it;
        }
    }

    publishMarkers(now, msg->header.frame_id);
  }

  void publishMarkers(rclcpp::Time now, std::string frame_id) {
      visualization_msgs::msg::MarkerArray candidate_array, stable_array, debug_array, goals_array;
      int id = 0;

      float r=0.0f, g=tracking_enabled_?1.0f:0.0f, b=tracking_enabled_?0.0f:1.0f;

      for (const auto &c : stable_objects_) {
          // 1. The Box
          visualization_msgs::msg::Marker m;
          m.header.frame_id = frame_id; m.header.stamp = now;
          m.ns = "stable"; m.id = id++; m.type = visualization_msgs::msg::Marker::CUBE;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.pose.position.x = c.cx; m.pose.position.y = c.cy; m.pose.position.z = 0.2;
          m.scale.x = c.width; m.scale.y = c.height; m.scale.z = 0.4;
          m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.8;
          stable_array.markers.push_back(m);

          // 2. The Lock Zone (Red Circle)
          visualization_msgs::msg::Marker zone;
          zone.header.frame_id = frame_id; zone.header.stamp = now;
          zone.ns = "lock_zone"; zone.id = id++; zone.type = visualization_msgs::msg::Marker::CYLINDER;
          zone.action = visualization_msgs::msg::Marker::ADD;
          zone.pose.position.x = c.cx; zone.pose.position.y = c.cy; zone.pose.position.z = 0.0;
          zone.scale.x = c.lock_radius * 2.0; zone.scale.y = c.lock_radius * 2.0; zone.scale.z = 0.05;
          zone.color.r = 1.0; zone.color.a = 0.1;
          debug_array.markers.push_back(zone);

          // 3. ★ VISITING POINTS with ORIENTATION ★
          float vp_radius = c.lock_radius + visiting_point_buffer_;
          std::vector<std::pair<float, float>> vps = {
              {c.cx, c.cy + vp_radius}, {c.cx, c.cy - vp_radius},
              {c.cx + vp_radius, c.cy}, {c.cx - vp_radius, c.cy}
          };

          for (size_t i=0; i<4; i++) {
              visualization_msgs::msg::Marker p;
              p.header.frame_id = frame_id; p.header.stamp = now;
              p.ns = "visiting_points"; 
              // ID Encoding: Object ID * 10 + Point Index
              int obj_id_index = &c - &stable_objects_[0]; 
              p.id = (obj_id_index * 10) + i; 

              p.type = visualization_msgs::msg::Marker::ARROW; // Changed to ARROW to see orientation
              p.action = visualization_msgs::msg::Marker::ADD;
              p.pose.position.x = vps[i].first; 
              p.pose.position.y = vps[i].second; 
              p.pose.position.z = 0.2;

              // --- CALCULATE ORIENTATION (Face Center) ---
              // Vector from Point to Center
              float dx = c.cx - vps[i].first;
              float dy = c.cy - vps[i].second;
              float yaw = std::atan2(dy, dx);

              // Manual Quaternion Conversion (Yaw -> Quat)
              p.pose.orientation.w = cos(yaw * 0.5);
              p.pose.orientation.z = sin(yaw * 0.5);
              p.pose.orientation.x = 0.0;
              p.pose.orientation.y = 0.0;
              // -------------------------------------------

              p.scale.x = 0.3; p.scale.y = 0.05; p.scale.z = 0.05; // Arrow size
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

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr object_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_pub_, stable_pub_, debug_pub_, goal_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service_;

  double cluster_distance_threshold_, wall_thickness_threshold_, stability_time_, lock_margin_, smoothing_factor_, visiting_point_buffer_;
  int min_cluster_points_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectClusterMarker>());
  rclcpp::shutdown();
  return 0;
}