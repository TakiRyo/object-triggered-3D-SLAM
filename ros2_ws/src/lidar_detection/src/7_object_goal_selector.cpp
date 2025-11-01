#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

struct TrackedCluster
{
  float min_x, max_x;
  float min_y, max_y;
  float cx, cy;
  float width, height;
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
  bool stable;
};

class ObjectClusterMarker : public rclcpp::Node
{
public:
  ObjectClusterMarker()
  : Node("object_goal_selector")
  {
    // --- Parameters ---
    this->declare_parameter("cluster_distance_threshold", 2.5);
    this->declare_parameter("min_cluster_points", 10);
    this->declare_parameter("wall_thickness_threshold", 0.2);
    this->declare_parameter("stability_time", 3.0);

    cluster_distance_threshold_ = this->get_parameter("cluster_distance_threshold").as_double();
    min_cluster_points_ = this->get_parameter("min_cluster_points").as_int();
    wall_thickness_threshold_ = this->get_parameter("wall_thickness_threshold").as_double();
    stability_time_ = this->get_parameter("stability_time").as_double();

    // --- Subscribers & Publishers ---
    object_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/object_clusters", 10,
      std::bind(&ObjectClusterMarker::cloudCallback, this, std::placeholders::_1));

    candidate_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/candidate_clusters", 10);
    stable_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/stable_clusters", 10);

    RCLCPP_INFO(this->get_logger(),
      "✅ ObjectGoalSelector started (dist=%.2f, min_pts=%d, wall=%.2f, stable=%.1fs)",
      cluster_distance_threshold_, min_cluster_points_, wall_thickness_threshold_, stability_time_);
  }

private:
  // --- Overlap check ---
  bool isOverlap(const TrackedCluster &a, const TrackedCluster &b)
  {
    bool x_overlap = (b.min_x <= a.max_x) && (b.max_x >= a.min_x);
    bool y_overlap = (b.min_y <= a.max_y) && (b.max_y >= a.min_y);
    return x_overlap && y_overlap;
  }

  // --- Inclusion check ---
  bool isIncluded(const TrackedCluster &a, const TrackedCluster &b)
  {
    bool x_included = (a.min_x >= b.min_x) && (a.max_x <= b.max_x);
    bool y_included = (a.min_y >= b.min_y) && (a.max_y <= b.max_y);
    return x_included && y_included;
  }

  // --- Main callback ---
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    rclcpp::Time now = this->get_clock()->now();

    // --- Step 1: Extract points ---
    std::vector<std::pair<float, float>> points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
      points.emplace_back(*iter_x, *iter_y);
    if (points.empty()) return;

    // --- Step 2: Create clusters (distance-based) ---
    std::vector<std::vector<std::pair<float, float>>> clusters;
    std::vector<std::pair<float, float>> current;
    current.push_back(points.front());
    for (size_t i = 1; i < points.size(); ++i)
    {
      float dx = points[i].first - points[i-1].first;
      float dy = points[i].second - points[i-1].second;
      float dist = std::sqrt(dx*dx + dy*dy);

      if (dist > cluster_distance_threshold_) {
        if (current.size() >= static_cast<size_t>(min_cluster_points_))
          clusters.push_back(current);
        current.clear();
      }
      current.push_back(points[i]);
    }
    if (current.size() >= static_cast<size_t>(min_cluster_points_))
      clusters.push_back(current);

    // --- Step 3: Convert to candidate TrackedClusters ---
    std::vector<TrackedCluster> candidates;
    for (const auto &cluster : clusters)
    {
      float min_x = std::numeric_limits<float>::max();
      float max_x = std::numeric_limits<float>::lowest();
      float min_y = std::numeric_limits<float>::max();
      float max_y = std::numeric_limits<float>::lowest();

      for (const auto &p : cluster)
      {
        min_x = std::min(min_x, p.first);
        max_x = std::max(max_x, p.first);
        min_y = std::min(min_y, p.second);
        max_y = std::max(max_y, p.second);
      }

      float cx = (min_x + max_x) / 2.0f;
      float cy = (min_y + max_y) / 2.0f;
      float width  = std::max(0.1f, max_x - min_x);
      float height = std::max(0.1f, max_y - min_y);

      // Skip thin wall-like clusters
      if (std::min(width, height) < wall_thickness_threshold_) continue;

      candidates.push_back({min_x, max_x, min_y, max_y, cx, cy, width, height, now, now, false});
    }

    // --- Step 4: Compare candidates with tracked stable clusters ---
    for (const auto &cand : candidates)
    {
      bool skip = false;
      for (auto &stable : tracked_clusters_)
      {
        if (isIncluded(cand, stable))
        {
          skip = true; // candidate inside stable
          break;
        }
        else if (isIncluded(stable, cand))
        {
          stable = cand; // stable inside candidate → replace (bigger one)
          skip = true;
          break;
        }
        else if (isOverlap(cand, stable))
        {
          skip = true; // overlap → keep stable
          break;
        }
      }
      if (!skip)
        tracked_clusters_.push_back(cand);
    }

    // --- Step 5: Update stability ---
    for (auto &t : tracked_clusters_)
    {
      if ((now - t.first_seen).seconds() > stability_time_)
        t.stable = true;
    }

    // --- Step 6: Publish both candidates and stable clusters ---
    visualization_msgs::msg::MarkerArray candidate_array, stable_array;
    int cid = 0, sid = 0;

    for (const auto &c : tracked_clusters_)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = msg->header.frame_id;
      marker.header.stamp = now;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = c.cx;
      marker.pose.position.y = c.cy;
      marker.pose.position.z = 0.1;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = c.width;
      marker.scale.y = c.height;
      marker.scale.z = 0.1;
      marker.lifetime = rclcpp::Duration::from_seconds(0);

      if (c.stable) {
        marker.ns = "stable_clusters";
        marker.id = sid++;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.7f;
        stable_array.markers.push_back(marker);
      } else {
        marker.ns = "candidate_clusters";
        marker.id = cid++;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.6f;
        candidate_array.markers.push_back(marker);
      }
    }

    candidate_pub_->publish(candidate_array);
    stable_pub_->publish(stable_array);
    RCLCPP_INFO(this->get_logger(), "🟨 %zu candidates | 🟩 %zu stable clusters",
                candidate_array.markers.size(), stable_array.markers.size());
  }

  // --- Members ---
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr object_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stable_pub_;
  double cluster_distance_threshold_;
  int min_cluster_points_;
  double wall_thickness_threshold_;
  double stability_time_;
  std::vector<TrackedCluster> tracked_clusters_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectClusterMarker>());
  rclcpp::shutdown();
  return 0;
}
