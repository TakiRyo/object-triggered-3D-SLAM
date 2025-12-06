#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class VirtualScanNode : public rclcpp::Node
{
public:
  VirtualScanNode()
  : Node("virtual_scan_node")
  {
    // 1. Subscribe to the Map (QoS is important for static maps!)
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/virtual_map", map_qos, std::bind(&VirtualScanNode::map_callback, this, std::placeholders::_1));
      
    // 2. Publisher for the Virtual Scan
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/virtual_scan", 10);

    // 3. TF Listener (To know where robot is)
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 4. Timer: Run the calculation at 10Hz
    timer_ = this->create_wall_timer(
      100ms, std::bind(&VirtualScanNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Virtual Scan Node has started.");
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // Store map only once
    current_map_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Map received: %d x %d", msg->info.width, msg->info.height);
  }

  void timer_callback()
  {
    if (current_map_.data.empty()) {
      return; // No map yet
    }

    // Get Robot Position (map -> base_link)
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
      return;
    }

    // --- TODO: RAYCASTING LOGIC GOES HERE ---
    // For now, we publish a fake scan to test connections
    publish_dummy_scan(t); 
  }

  void publish_dummy_scan(const geometry_msgs::msg::TransformStamped & transform)
  {
    auto scan = sensor_msgs::msg::LaserScan();
    scan.header.stamp = this->now();
    scan.header.frame_id = "base_scan"; // Should match your real LiDAR frame
    
    // Standard LiDAR settings (e.g., 360 degrees)
    scan.angle_min = -3.14;
    scan.angle_max = 3.14;
    scan.angle_increment = 3.14 / 180.0; // 1 degree res
    scan.range_min = 0.1;
    scan.range_max = 10.0;
    
    // Fill with fake data (2.0 meters everywhere)
    int num_readings = (scan.angle_max - scan.angle_min) / scan.angle_increment;
    scan.ranges.assign(num_readings, 2.0); 

    scan_pub_->publish(scan);
  }

  // Member variables
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::OccupancyGrid current_map_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualScanNode>());
  rclcpp::shutdown();
  return 0;
}