#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <mutex>

class ObjectMapFilter : public rclcpp::Node
{
public:
  ObjectMapFilter() : Node("object_map_filter")
  {
    // The "Safety Radius". If an object is within 0.5m of a wall, ignore it.
    this->declare_parameter("proximity_threshold", 0.5);
    this->get_parameter("proximity_threshold", proximity_threshold_);

    // 1. Subscribe to Object Clusters (Your Blue Points)
    // Ensure this matches the topic from LidarClusterPublisher
    cluster_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_object_clusters", 10, std::bind(&ObjectMapFilter::clusterCallback, this, std::placeholders::_1));

    // 2. Subscribe to Virtual Scan (To know where walls are)
    virtual_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/virtual_scan", 10, std::bind(&ObjectMapFilter::scanCallback, this, std::placeholders::_1));

    // 3. Publish Filtered Objects
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_objects", 10);

    // TF Setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Object Map Filter Started. Radius: %.2f m", proximity_threshold_);
  }

private:
  // Store the latest wall points in Map Frame
  std::vector<geometry_msgs::msg::Point> map_wall_points_;
  std::mutex map_mutex_;
  double proximity_threshold_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // --- 1. Process Virtual Scan into a List of Wall Points ---
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::vector<geometry_msgs::msg::Point> temp_walls;

    // Get Transform (Base_Scan -> Map)
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("map", msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      return; 
    }

    // Convert Ranges to Points (Map Frame)
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float r = msg->ranges[i];
      if (std::isinf(r) || std::isnan(r) || r > msg->range_max) continue;

      float angle = msg->angle_min + i * msg->angle_increment;
      
      // Point in Laser Frame
      float lx = r * std::cos(angle);
      float ly = r * std::sin(angle);

      // Transform to Map Frame
      geometry_msgs::msg::Point p_map = transformPoint(lx, ly, t);
      temp_walls.push_back(p_map);
    }

    // Update the shared list
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_wall_points_ = temp_walls;
    }
  }

  // --- 2. Filter Object Clusters ---
  void clusterCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Retrieve Wall Points safely
    std::vector<geometry_msgs::msg::Point> walls;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (map_wall_points_.empty()) return; // No map data yet
        walls = map_wall_points_;
    }

    // Prepare Output
    auto out_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out_cloud->header = msg->header; // Should be "map"
    out_cloud->height = 1;
    sensor_msgs::PointCloud2Modifier modifier(*out_cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    // Iterate Input Points
    sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(*msg, "z");

    std::vector<geometry_msgs::msg::Point> valid_points;

    // Check every Object Point
    for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z) {
        float ox = *in_x;
        float oy = *in_y;

        // --- PROXIMITY CHECK ---
        bool near_wall = false;
        
        // Simple search (Optimization: A KD-Tree would be faster, but for <2000 points this is fine)
        for (const auto& w : walls) {
            float dx = ox - w.x;
            float dy = oy - w.y;
            // Squared distance check is faster
            if ((dx*dx + dy*dy) < (proximity_threshold_ * proximity_threshold_)) {
                near_wall = true;
                break;
            }
        }

        // Only keep if FAR from any wall
        if (!near_wall) {
            geometry_msgs::msg::Point p;
            p.x = ox; p.y = oy; p.z = *in_z;
            valid_points.push_back(p);
        }
    }

    // Publish
    if (!valid_points.empty()) {
        modifier.resize(valid_points.size());
        sensor_msgs::PointCloud2Iterator<float> out_x(*out_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(*out_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(*out_cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_r(*out_cloud, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_g(*out_cloud, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_b(*out_cloud, "b");

        for (const auto &p : valid_points) {
            *out_x = p.x; *out_y = p.y; *out_z = p.z;
            *out_r = 0; *out_g = 255; *out_b = 0; // Green for Final Valid Objects
            ++out_x; ++out_y; ++out_z; ++out_r; ++out_g; ++out_b; 
        }
        filtered_pub_->publish(std::move(out_cloud));
    }
  }

  geometry_msgs::msg::Point transformPoint(float x, float y, const geometry_msgs::msg::TransformStamped& t) {
      geometry_msgs::msg::Point p;
      // Manual rotation/translation for 2D (faster than tf2::doTransform for simple points)
      double qx = t.transform.rotation.x;
      double qy = t.transform.rotation.y;
      double qz = t.transform.rotation.z;
      double qw = t.transform.rotation.w;

      // Yaw from quaternion
      double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

      p.x = t.transform.translation.x + (x * std::cos(yaw) - y * std::sin(yaw));
      p.y = t.transform.translation.y + (x * std::sin(yaw) + y * std::cos(yaw));
      p.z = 0.0;
      return p;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr virtual_scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectMapFilter>());
  rclcpp::shutdown();
  return 0;
}