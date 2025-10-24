#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

class LidarClusterPublisher : public rclcpp::Node
{
public:
  LidarClusterPublisher()
  : Node("lidar_cluster_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&LidarClusterPublisher::scanCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/object_clusters", 10);

    gap_threshold_ = 0.1; //minimam distance to make cluster.
    min_cluster_points_ = 0; //minimam cluster points. 

    RCLCPP_INFO(this->get_logger(), "✅ LidarClusterPublisher started (cluster visualization)");
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

    // --- Transform to map frame ---
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF not ready: %s", ex.what());
      return;
    }

    // --- Prepare PointCloud2 ---
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.height = 1;

    // estimate total points
    size_t total_points = 0;
    for (auto &c : clusters) total_points += c.size();
    cloud_msg.width = total_points;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(total_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

    // --- Fill points ---
    int cluster_id = 0;
    for (auto &cluster : clusters)
    {
      // random-ish color per cluster
      uint8_t r = (cluster_id * 70) % 255;
      uint8_t g = (cluster_id * 150) % 255;
      uint8_t b = (cluster_id * 230) % 255;
      cluster_id++;

      for (auto &p : cluster)
      {
        geometry_msgs::msg::PoseStamped pt_base, pt_map;
        pt_base.header.frame_id = "base_link";
        pt_base.pose.position.x = p.first;
        pt_base.pose.position.y = p.second;
        pt_base.pose.orientation.w = 1.0;
        tf2::doTransform(pt_base, pt_map, transform);

        *iter_x = static_cast<float>(pt_map.pose.position.x);
        *iter_y = static_cast<float>(pt_map.pose.position.y);
        *iter_z = 0.0f;

        *iter_r = r;
        *iter_g = g;
        *iter_b = b;

        ++iter_x; ++iter_y; ++iter_z;
        ++iter_r; ++iter_g; ++iter_b;
      }
    }

    cloud_pub_->publish(cloud_msg);
  }

  // --- Members ---
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  float gap_threshold_;
  int min_cluster_points_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarClusterPublisher>());
  rclcpp::shutdown();
  return 0;
}
