#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
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

    wall_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/wall_clusters", 10);
    object_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/object_clusters", 10);

    gap_threshold_ = 0.1; 
    min_cluster_points_ = 1;

    RCLCPP_INFO(this->get_logger(), "✅ LidarClusterPublisher started (publishes wall + object clouds)");
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

  // ---------- helper: compute linearity (PCA) ----------
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
    if (eigvals(0) + eigvals(1) < 1e-6) return 0.0f;
    float ratio = eigvals(0) / (eigvals(1) + 1e-6); // small/large
    return ratio; // near 0 = line-like, near 1 = blob
  }

  // ---------- main callback ----------
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Step 1. Split scan into clusters
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

    // Step 2. Transform to map frame
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF not ready: %s", ex.what());
      return;
    }

    // Separate points into wall vs object lists
    std::vector<geometry_msgs::msg::Point> wall_points;
    std::vector<geometry_msgs::msg::Point> object_points;

    for (auto &cluster : clusters)
    {
      float length = computeClusterLength(cluster);
      float linearity = computeLinearity(cluster);

      bool is_wall = (length > 1.0 && linearity < 0.2);

      for (auto &p : cluster)
      {
        geometry_msgs::msg::PoseStamped pt_base, pt_map;
        pt_base.header.frame_id = "base_link";
        pt_base.pose.position.x = p.first;
        pt_base.pose.position.y = p.second;
        pt_base.pose.orientation.w = 1.0;
        tf2::doTransform(pt_base, pt_map, transform);

        geometry_msgs::msg::Point pt;
        pt.x = pt_map.pose.position.x;
        pt.y = pt_map.pose.position.y;
        pt.z = 0.0;
        if (is_wall)
          wall_points.push_back(pt);
        else
          object_points.push_back(pt);
      }
    }

    // Publish two point clouds
    publishPointCloud(wall_points, wall_pub_, 0, 0, 255);   // blue
    publishPointCloud(object_points, object_pub_, 255, 0, 0); // red
  }

  // ---------- helper: publish one cloud ----------
  void publishPointCloud(const std::vector<geometry_msgs::msg::Point>& points,
                         rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                         uint8_t r, uint8_t g, uint8_t b)
  {
    if (points.empty()) return;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.height = 1;
    cloud_msg.width = points.size();

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

    for (auto &p : points)
    {
      *iter_x = static_cast<float>(p.x);
      *iter_y = static_cast<float>(p.y);
      *iter_z = static_cast<float>(p.z);
      *iter_r = r; *iter_g = g; *iter_b = b;
      ++iter_x; ++iter_y; ++iter_z;
      ++iter_r; ++iter_g; ++iter_b;
    }

    pub->publish(cloud_msg);
  }

  // --- Members ---
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wall_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pub_;
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
