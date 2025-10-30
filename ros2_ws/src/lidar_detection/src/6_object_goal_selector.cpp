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
  : Node("object_cluster_marker")
  {
    // --- Parameters ---
    this->declare_parameter("cluster_distance_threshold", 2.5);
    this->declare_parameter("min_cluster_points", 10);
    this->declare_parameter("wall_thickness_threshold", 0.2); //0.2 previously
    this->declare_parameter("stability_time", 3.0); // seconds

    cluster_distance_threshold_ = this->get_parameter("cluster_distance_threshold").as_double();
    min_cluster_points_ = this->get_parameter("min_cluster_points").as_int();
    wall_thickness_threshold_ = this->get_parameter("wall_thickness_threshold").as_double();
    stability_time_ = this->get_parameter("stability_time").as_double();

    // --- Subscriber & Publisher ---
    object_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/object_clusters", 10,
      std::bind(&ObjectClusterMarker::cloudCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/stable_clusters", 10);

    RCLCPP_INFO(this->get_logger(),
      "ObjectClusterMarker started (dist_thresh=%.2f, min_points=%d, wall_thickness=%.2f, stability=%.1fs)",
      cluster_distance_threshold_, min_cluster_points_, wall_thickness_threshold_, stability_time_);
  }

private:
//   --- Overlap check ---
//   bool isOverlap(const TrackedCluster &a, const TrackedCluster &b)
//   {
//     return (a.min_x <= b.max_x && a.max_x >= b.min_x &&
//             a.min_y <= b.max_y && a.max_y >= b.min_y);
//   }
  bool isOverlap(const TrackedCluster &a, const TrackedCluster &b)
    {
    // Check overlap along X axis
    bool x_overlap = (b.min_x <= a.max_x) && (b.max_x >= a.min_x);

    // Check overlap along Y axis
    bool y_overlap = (b.min_y <= a.max_y) && (b.max_y >= a.min_y);

    // If both overlap, clusters overlap
    return x_overlap && y_overlap;
    }

  // --- Inclusion check (a inside b) ---
    bool isIncluded(const TrackedCluster &a, const TrackedCluster &b)
    {
    bool x_included = (a.min_x >= b.min_x) && (a.max_x <= b.max_x);
    bool y_included = (a.min_y >= b.min_y) && (a.max_y <= b.max_y);
    return x_included || y_included;
    }

  // --- Main callback ---
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    rclcpp::Time now = this->get_clock()->now();

    // --- Extract (x, y) points ---
    std::vector<std::pair<float, float>> points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
      points.emplace_back(*iter_x, *iter_y);
    if (points.empty()) return;

    // --- Step 1: Cluster points by distance ---
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

    // --- Step 2: Process each cluster ---
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

      // ✅ Skip thin wall-like cluster
      if (std::min(width, height) < wall_thickness_threshold_)
        continue;

      // --- Check if overlaps with existing tracked clusters ---
      bool matched = false;
      for (auto &t : tracked_clusters_)
      {
        if (isOverlap(t, {min_x, max_x, min_y, max_y, cx, cy, width, height, now, now, false}))
        {
          t.last_seen = now;
          t.width = 0.8f * t.width + 0.2f * width;   // smooth update
          t.height = 0.8f * t.height + 0.2f * height;
          t.cx = 0.8f * t.cx + 0.2f * cx;
          t.cy = 0.8f * t.cy + 0.2f * cy;
          matched = true;
          break;
        }
      }

      if (!matched)
      {
        tracked_clusters_.push_back({min_x, max_x, min_y, max_y, cx, cy, width, height, now, now, false});
      }
    }

    // --- Step 3: Mark stable clusters ---
    for (auto &t : tracked_clusters_)
      if ((now - t.first_seen).seconds() > stability_time_)
        t.stable = true;

    // --- Step 4: Remove overlapped or included smaller clusters ---
    std::vector<TrackedCluster> filtered;
    for (size_t i = 0; i < tracked_clusters_.size(); ++i)
    {
      bool skip = false;
      for (size_t j = 0; j < tracked_clusters_.size(); ++j)
      {
        if (i == j) continue;
        const auto &a = tracked_clusters_[i];
        const auto &b = tracked_clusters_[j];
        if (isIncluded(a, b) &&
            (b.width * b.height) > (a.width * a.height))
        {
          skip = true;
          break;
        }
      }
      if (!skip && tracked_clusters_[i].stable)
        filtered.push_back(tracked_clusters_[i]);
    }

    // --- Step 5: Publish markers for filtered clusters ---
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto &c : filtered)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = msg->header.frame_id;
      marker.header.stamp = now;
      marker.ns = "stable_clusters";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = c.cx;
      marker.pose.position.y = c.cy;
      marker.pose.position.z = 0.1;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = c.width;
      marker.scale.y = c.height;
      marker.scale.z = 0.1;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.7f;
      marker.lifetime = rclcpp::Duration::from_seconds(0);
      marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Published %zu stable clusters", marker_array.markers.size());
  }

  // --- Members ---
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr object_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
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
