#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

class LidarObjectDetector : public rclcpp::Node
{
public:
  LidarObjectDetector()
  : Node("lidar_object_detector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&LidarObjectDetector::scanCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_markers", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/object_positions", 10);

    gap_threshold_ = 0.3;         // distance jump for new cluster (m)
    min_cluster_points_ = 0;

    RCLCPP_INFO(this->get_logger(), "✅ LidarObjectDetector node started (C++ version)");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::vector<std::vector<std::pair<float, float>>> clusters;
    std::vector<std::pair<float, float>> current_cluster;
    float prev_range = std::numeric_limits<float>::quiet_NaN();

    // --- Split scan into clusters ---
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      float r = msg->ranges[i];
      if (std::isnan(r) || std::isinf(r)) continue;

      if (!std::isnan(prev_range) && std::fabs(r - prev_range) > gap_threshold_)
      {
        if (current_cluster.size() >= static_cast<size_t>(min_cluster_points_))
          clusters.push_back(current_cluster);
        current_cluster.clear();
      }

      float angle = msg->angle_min + i * msg->angle_increment;
      float x = r * std::cos(angle);
      float y = r * std::sin(angle);
      current_cluster.push_back({x, y});

      prev_range = r;
    }
    if (current_cluster.size() >= static_cast<size_t>(min_cluster_points_))
      clusters.push_back(current_cluster);

    // --- Compute centroids ---
    std::vector<std::pair<float, float>> centroids;
    for (auto &c : clusters)
    {
      float sx = 0.0f, sy = 0.0f;
      for (auto &p : c)
      {
        sx += p.first;
        sy += p.second;
      }
      sx /= c.size();
      sy /= c.size();
      centroids.push_back({sx, sy});
    }

    // --- Transform to map frame ---
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF not ready: %s", ex.what());
      return;
    }

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = this->get_clock()->now();

    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    for (auto &c : centroids)
    {
      geometry_msgs::msg::PoseStamped pose_base, pose_map;
      pose_base.header.frame_id = "base_link";
      pose_base.pose.position.x = c.first;
      pose_base.pose.position.y = c.second;
      pose_base.pose.orientation.w = 1.0;

      tf2::doTransform(pose_base, pose_map, transform);
      pose_array.poses.push_back(pose_map.pose);

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = this->get_clock()->now();
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = m.scale.y = m.scale.z = 0.2;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 0.9;
      m.pose = pose_map.pose;
      marker_array.markers.push_back(m);
    }

    pose_pub_->publish(pose_array);
    marker_pub_->publish(marker_array);
  }

  // --- Members ---
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  float gap_threshold_;
  int min_cluster_points_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarObjectDetector>());
  rclcpp::shutdown();
  return 0;
}
