#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

class ObjectClusterMarker : public rclcpp::Node
{
public:
  ObjectClusterMarker()
  : Node("object_cluster_marker")
  {
    // Parameters
    this->declare_parameter("cluster_distance_threshold", 2.5); //PARAM
    cluster_distance_threshold_ = this->get_parameter("cluster_distance_threshold").as_double();

    // Subscriber & Publisher
    object_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/object_clusters", 10,
      std::bind(&ObjectClusterMarker::cloudCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/object_markers", 10);

    RCLCPP_INFO(this->get_logger(), "✅ ObjectClusterMarker started (threshold=%.2f m)",
                cluster_distance_threshold_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Extract (x, y) points from the PointCloud2
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
        if (!current.empty()) clusters.push_back(current);
        current.clear();
      }
      current.push_back(points[i]);
    }
    if (!current.empty()) clusters.push_back(current);

    // --- Step 2: Create marker array ---
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto &cluster : clusters)
    {
      if (cluster.size() < 1) continue; // skip tiny noise

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

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = msg->header.frame_id;
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "object_clusters";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = cx;
      marker.pose.position.y = cy;
      marker.pose.position.z = 0.1;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = width;
      marker.scale.y = height;
      marker.scale.z = 0.1;

      // 🟥 same color for all (red)
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.6f;

      marker.lifetime = rclcpp::Duration::from_seconds(0); // persistent

      marker_array.markers.push_back(marker);
    }

    // --- Step 3: Publish markers ---
    marker_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "📦 Published %zu clusters as markers", clusters.size());
  }

  // --- Members ---
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr object_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  double cluster_distance_threshold_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectClusterMarker>());
  rclcpp::shutdown();
  return 0;
}
