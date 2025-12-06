/**
 * ------------------------------------------------------------
 * LiDAR Cluster Classification Node (Enhanced with Virtual Scan)
 * ------------------------------------------------------------
 * 1. Subtraction: Removes points that exist in Virtual Map.
 * 2. Clustering: Groups the remaining "New" points.
 * 3. Classification: Checks size/linearity to confirm Object vs Noise.
 * ------------------------------------------------------------
 */

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
#include <mutex>

class LidarClusterPublisher : public rclcpp::Node
{
public:
  LidarClusterPublisher()
  : Node("lidar_cluster_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ---- Parameters ----
    this->declare_parameter("gap_threshold", 0.2);
    this->declare_parameter("min_cluster_points", 1);
    this->declare_parameter("max_range_ratio", 1.0);
    this->declare_parameter("diff_threshold", 1.0); // 30cm difference needed to be "New"

    // Classification Limits
    this->declare_parameter("wal_len_min", 2.0);
    this->declare_parameter("wal_lin_max", 0.02); // Slightly relaxed for real world
    this->declare_parameter("wal_nmp_min", 15);
    this->declare_parameter("obj_len_max", 1.2);  // Max size for a "visiting target"
    this->declare_parameter("obj_nmp_min", 2);

    // Get parameters
    this->get_parameter("gap_threshold", gap_threshold_);
    this->get_parameter("min_cluster_points", min_cluster_points_);
    this->get_parameter("max_range_ratio", max_range_ratio_);
    this->get_parameter("diff_threshold", diff_threshold_);
    
    this->get_parameter("wal_len_min", wal_len_min_);
    this->get_parameter("wal_lin_max", wal_lin_max_);
    this->get_parameter("wal_nmp_min", wal_nmp_min_);
    this->get_parameter("obj_len_max", obj_len_max_);
    this->get_parameter("obj_nmp_min", obj_nmp_min_);

    // ---- Subscribers ----
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LidarClusterPublisher::scanCallback, this, std::placeholders::_1));
    
    virtual_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/virtual_scan", 10, std::bind(&LidarClusterPublisher::virtualScanCallback, this, std::placeholders::_1));

    // ---- Publishers ----
    wall_pub_    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/wall_clusters", 10);
    object_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/object_clusters", 10);
    unknown_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/unknown_clusters", 10);
    static_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/static_environment", 10);
  }

private:
  sensor_msgs::msg::LaserScan::SharedPtr last_virtual_scan_;
  std::mutex virtual_scan_mutex_;

  void virtualScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(virtual_scan_mutex_);
    last_virtual_scan_ = msg;
  }

  // ---------- Helper: Compute Cluster Length ----------
  float computeClusterLength(const std::vector<std::pair<float,float>>& cluster)
  {
    float min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
    for (auto &p : cluster) {
      min_x = std::min(min_x, p.first); max_x = std::max(max_x, p.first);
      min_y = std::min(min_y, p.second); max_y = std::max(max_y, p.second);
    }
    return std::sqrt(std::pow(max_x - min_x, 2) + std::pow(max_y - min_y, 2));
  }

  // ---------- Helper: Compute Linearity (PCA) ----------
  float computeLinearity(const std::vector<std::pair<float,float>>& cluster)
  {
    if (cluster.size() < 3) return 0.0f;
    Eigen::MatrixXf points(2, cluster.size());
    for (size_t i=0; i<cluster.size(); ++i) {
      points(0,i) = cluster[i].first;
      points(1,i) = cluster[i].second;
    }
    Eigen::Vector2f mean = points.rowwise().mean();
    for (size_t i=0; i<cluster.size(); ++i) points.col(i) -= mean;
    Eigen::Matrix2f cov = (points * points.transpose()) / (cluster.size()-1);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
    Eigen::Vector2f eigvals = solver.eigenvalues();
    // linearity: 0.0 = line, 1.0 = circle/blob
    if (eigvals(0) + eigvals(1) < 1e-6) return 0.0f;
    return eigvals(0) / (eigvals(1) + 1e-6);
  }

  // ---------- Main Callback ----------
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // 1. Get Virtual Scan
    sensor_msgs::msg::LaserScan::SharedPtr v_scan;
    {
        std::lock_guard<std::mutex> lock(virtual_scan_mutex_);
        if (!last_virtual_scan_) return; 
        v_scan = last_virtual_scan_;
    }

    if (msg->ranges.size() != v_scan->ranges.size()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Scan mismatch!");
        return;
    }

    float max_use_range = msg->range_max * max_range_ratio_;

    // Containers
    std::vector<std::vector<std::pair<float, float>>> clusters;
    std::vector<std::pair<float, float>> current_cluster;
    std::vector<std::pair<float, float>> static_raw_points; // For debugging

    // 2. Iterate and Filter (Subtraction)
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      float r_real = msg->ranges[i];
      float r_virt = v_scan->ranges[i];

      if (std::isnan(r_real) || std::isinf(r_real) || r_real > max_use_range) continue;

      float angle = msg->angle_min + i * msg->angle_increment;
      float x = r_real * std::cos(angle);
      float y = r_real * std::sin(angle);

      // --- LOGIC: IS THIS NEW? ---
      // If Real is much smaller than Virtual, it is a NEW object.
      // If Real ~= Virtual, it is the WALL (Static).
      if (r_real > (r_virt - diff_threshold_)) {
          // It's part of the map. Add to static list.
          static_raw_points.push_back({x, y});

          // End current dynamic cluster
          if (!current_cluster.empty()) {
             if (current_cluster.size() >= static_cast<size_t>(min_cluster_points_))
                clusters.push_back(current_cluster);
             current_cluster.clear();
          }
          continue; 
      }

      // --- LOGIC: CLUSTERING DYNAMIC POINTS ---
      if (!current_cluster.empty())
      {
        auto [prev_x, prev_y] = current_cluster.back();
        float dist = std::hypot(x - prev_x, y - prev_y);
        if (dist > gap_threshold_)
        {
          if (current_cluster.size() >= static_cast<size_t>(min_cluster_points_))
            clusters.push_back(current_cluster);
          current_cluster.clear();
        }
      }
      current_cluster.push_back({x, y});
    }
    // Add last cluster
    if (current_cluster.size() >= static_cast<size_t>(min_cluster_points_))
        clusters.push_back(current_cluster);

    // Merge wrap-around
    if (!clusters.empty()) {
        auto &first = clusters.front();
        auto &last  = clusters.back();
        if (!first.empty() && !last.empty()) {
            auto [x1, y1] = last.back();
            auto [x2, y2] = first.front();
            if (std::hypot(x1 - x2, y1 - y2) < gap_threshold_) {
              last.insert(last.end(), first.begin(), first.end());
              clusters.erase(clusters.begin());
            }
        }
    }

    // --- TF Lookup ---
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      return;
    }

    // --- 3. CLASSIFY CLUSTERS (Geometric Check) ---
    std::vector<geometry_msgs::msg::Point> object_points;   // BLUE
    std::vector<geometry_msgs::msg::Point> wall_points;     // GREEN (Should be rare here)
    std::vector<geometry_msgs::msg::Point> unknown_points;  // YELLOW
    std::vector<geometry_msgs::msg::Point> static_points;   // RED

    for (const auto &cluster : clusters)
    {
      float length = computeClusterLength(cluster);
      float linearity = computeLinearity(cluster);
      size_t n_points = cluster.size();

      std::string type = "UNKNOWN";

      // A. Is it a long straight line? (Maybe a moving wall or door?)
      if (linearity < wal_lin_max_ && length > wal_len_min_ && n_points > wal_nmp_min_) {
        type = "WALL"; 
      }
      // B. Is it a small compact object? (Person, Box, Robot)
      else if (length < obj_len_max_ && n_points > obj_nmp_min_) {
        type = "OBJECT";
      }

      // Transform points
      for (auto &p : cluster) {
        geometry_msgs::msg::Point pt = transformPoint(p.first, p.second, transform);
        if (type == "OBJECT") object_points.push_back(pt);
        else if (type == "WALL") wall_points.push_back(pt);
        else unknown_points.push_back(pt);
      }
    }

    // Process Static Points (Just transform)
    for (auto &p : static_raw_points) {
        static_points.push_back(transformPoint(p.first, p.second, transform));
    }

    // --- Publish ---
    publishPointCloud(object_points, object_pub_, 0, 0, 255);       // Blue (Valid Objects)
    publishPointCloud(wall_points, wall_pub_, 0, 255, 0);           // Green (Weird long changes)
    publishPointCloud(unknown_points, unknown_pub_, 255, 255, 0);   // Yellow (Noise/Blobs)
    publishPointCloud(static_points, static_pub_, 255, 0, 0);       // Red (Known Map)
  }

  // Helper: Transform
  geometry_msgs::msg::Point transformPoint(float x, float y, const geometry_msgs::msg::TransformStamped& t) {
      geometry_msgs::msg::PoseStamped pt_base, pt_map;
      pt_base.header.frame_id = "base_link";
      pt_base.pose.position.x = x;
      pt_base.pose.position.y = y;
      pt_base.pose.orientation.w = 1.0;
      tf2::doTransform(pt_base, pt_map, t);
      geometry_msgs::msg::Point pt;
      pt.x = pt_map.pose.position.x;
      pt.y = pt_map.pose.position.y;
      pt.z = 0.0;
      return pt;
  }

  void publishPointCloud(const std::vector<geometry_msgs::msg::Point>& points,
                         rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                         uint8_t r, uint8_t g, uint8_t b)
  {
    if (points.empty()) return;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.height = 1; cloud_msg.width = points.size();
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> ix(cloud_msg, "x"), iy(cloud_msg, "y"), iz(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> ir(cloud_msg, "r"), ig(cloud_msg, "g"), ib(cloud_msg, "b");

    for (auto &p : points) {
      *ix = p.x; *iy = p.y; *iz = p.z;
      *ir = r; *ig = g; *ib = b;
      ++ix; ++iy; ++iz; ++ir; ++ig; ++ib;
    }
    pub->publish(cloud_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr virtual_scan_sub_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wall_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  float gap_threshold_;
  int   min_cluster_points_;
  float max_range_ratio_;
  float diff_threshold_;

  float obj_len_max_;
  float wal_len_min_;
  float wal_lin_max_;
  int obj_nmp_min_;
  int wal_nmp_min_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarClusterPublisher>());
  rclcpp::shutdown();
  return 0;
}