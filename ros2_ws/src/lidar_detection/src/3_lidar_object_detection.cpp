#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <Eigen/Dense>   // for PCA

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

    gap_threshold_ = 0.1;      // distance gap (m)
    min_cluster_points_ = 5;   // minimum cluster size

    RCLCPP_INFO(this->get_logger(), "✅ LidarClusterPublisher with wall filtering started");
  }

private:
  // ---------- helper: compute cluster length ----------
  float computeClusterLength(const std::vector<std::pair<float,float>>& cluster)
  {
    float min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
    for (auto &p : cluster)
    {
      min_x = std::min(min_x, p.first);
      max_x = std::max(max_x, p.first);
      min_y = std::min(min_y, p.second);
      max_y = std::max(max_y, p.second);
    }
    float dx = max_x - min_x;
    float dy = max_y - min_y;
    return std::sqrt(dx*dx + dy*dy);
  }

  // ---------- helper: compute linearity via PCA ----------
  float computeLinearity(const std::vector<std::pair<float,float>>& cluster)
  {
    if (cluster.size() < 3) return 0.0f;
    Eigen::MatrixXf points(2, cluster.size());
    for (size_t i=0; i<cluster.size(); ++i)
    {
      points(0,i) = cluster[i].first;
      points(1,i) = cluster[i].second;
    }
    Eigen::Vector2f mean = points.rowwise().mean();
    for (size_t i=0; i<cluster.size(); ++i)
      points.col(i) -= mean;
    Eigen::Matrix2f cov = (points * points.transpose()) / (cluster.size()-1);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
    Eigen::Vector2f eigvals = solver.eigenvalues();
    if (eigvals(1) < 1e-6) return 0.0f;
    float ratio = eigvals(1) / (eigvals(0) + 1e-6);  // small/large
    return ratio; // ~0 for line-like, ~1 for blob
  }

  // ---------- main callback ----------
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // --- Step 1. Split scan into clusters ---
    std::vector<std::vector<std::pair<float, float>>> clusters;
    std::vector<std::pair<float, float>> current_cluster;
    float prev_range = std::numeric_limits<float>::quiet_NaN();

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
      current_cluster.push_back({r * std::cos(angle), r * std::sin(angle)});
      prev_range = r;
    }
    if (current_cluster.size() >= static_cast<size_t>(min_cluster_points_))
      clusters.push_back(current_cluster);

    // --- Step 2. TF to map frame ---
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF not ready: %s", ex.what());
      return;
    }

    // --- Step 3. Prepare PointCloud2 ---
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.height = 1;

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

    // --- Step 4. Filter & color clusters ---
    for (auto &cluster : clusters)
    {
      float length = computeClusterLength(cluster);
      float linearity = computeLinearity(cluster);

      bool is_wall = (length > 1.0 && linearity < 0.2); // long + linear → wall

      uint8_t r = is_wall ? 0   : 255; // red = object
      uint8_t g = 0;
      uint8_t b = is_wall ? 255 : 0;   // blue = wall

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
        *iter_r = r; *iter_g = g; *iter_b = b;
        ++iter_x; ++iter_y; ++iter_z;
        ++iter_r; ++iter_g; ++iter_b;
      }
    }

    cloud_pub_->publish(cloud_msg);
  }

  // ---------- members ----------
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
