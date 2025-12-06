// #include <chrono>
// #include <cmath>
// #include <memory>
// #include <algorithm> // Added for bounds checking
// #include <limits>    // Added for infinity
// #include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "tf2/utils.h" // Added for getYaw
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// using namespace std::chrono_literals;

// class VirtualScanNode : public rclcpp::Node
// {
// public:
//   VirtualScanNode()
//   : Node("virtual_scan_node")
//   {
//     // 1. Subscribe to the Map
//     auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
//     map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//       "/virtual_map", map_qos, std::bind(&VirtualScanNode::map_callback, this, std::placeholders::_1));
      
//     // 2. Publisher for the Virtual Scan
//     scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/virtual_scan", 10);

//     // 3. TF Listener
//     tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//     // 4. Timer: Run at 10Hz
//     timer_ = this->create_wall_timer(
//       100ms, std::bind(&VirtualScanNode::timer_callback, this));

//     RCLCPP_INFO(this->get_logger(), "Virtual Scan Node has started.");
//   }

// private:
//   void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
//   {
//     current_map_ = *msg;
//     map_received_ = true;
//     RCLCPP_INFO(this->get_logger(), "Map received: %d x %d", msg->info.width, msg->info.height);
//   }

//   void timer_callback()
//   {
//     if (!map_received_) {
//       return; // No map yet
//     }

//     // Get Robot Position
//     geometry_msgs::msg::TransformStamped t;
//     try {
//       t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
//     } catch (const tf2::TransformException & ex) {
//       // It is normal to fail initially
//       return;
//     }

//     // Call the REAL logic function
//     publish_virtual_scan(t); 
//   }

//   void publish_virtual_scan(const geometry_msgs::msg::TransformStamped & transform)
//   {
//     auto scan = sensor_msgs::msg::LaserScan();
//     scan.header.stamp = this->now();
//     scan.header.frame_id = "base_scan"; 
    
//     // --- 1. SETUP LIDAR PARAMETERS (Based on your XML) ---
//     scan.angle_min = 0.0;
//     scan.angle_max = 6.2800;
//     scan.range_min = 0.3;
//     scan.range_max = 10.0;
    
//     // Calculate increment to match 1440 samples exactly
//     // (6.2800 - 0.0) / 1440 = ~0.00436
//     scan.angle_increment = (scan.angle_max - scan.angle_min) / 1440.0;
//     scan.time_increment = 0.0;
//     scan.scan_time = 0.1; // 10Hz

//     // Resize array
//     int num_readings = 1440;
//     scan.ranges.assign(num_readings, std::numeric_limits<float>::infinity());

//     // --- 2. PREPARE MATH ---
//     // Robot Pose
//     double robot_x = transform.transform.translation.x;
//     double robot_y = transform.transform.translation.y;
//     double robot_yaw = tf2::getYaw(transform.transform.rotation);

//     // Map Metadata
//     float resolution = current_map_.info.resolution;
//     float origin_x = current_map_.info.origin.position.x;
//     float origin_y = current_map_.info.origin.position.y;
//     int width = current_map_.info.width;
//     int height = current_map_.info.height;

//     // --- 3. RAY CASTING LOOP ---
//     for (int i = 0; i < num_readings; ++i) {
//         double angle = scan.angle_min + i * scan.angle_increment;
//         double global_angle = robot_yaw + angle; // Add robot's rotation

//         double ray_x = robot_x;
//         double ray_y = robot_y;
//         double dist = 0.0;
//         double step = resolution; // Move 1 pixel at a time

//         // Shoot the ray!
//         while (dist < scan.range_max) {
//             dist += step;
            
//             // Calculate point on ray
//             ray_x = robot_x + dist * cos(global_angle);
//             ray_y = robot_y + dist * sin(global_angle);

//             // Convert to Grid Index
//             int grid_x = (int)((ray_x - origin_x) / resolution);
//             int grid_y = (int)((ray_y - origin_y) / resolution);

//             // Check if outside map
//             if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
//                 break; // Stop ray, it went off the map
//             }

//             // Check if Wall (100)
//             int index = grid_y * width + grid_x;
//             if (current_map_.data[index] == 100) {
//                 scan.ranges[i] = dist; // Hit!
//                 break; // Stop this ray
//             }
//         }
//     }

//     scan_pub_->publish(scan);
//   }

//   // Member variables
//   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
//   rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
//   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//   nav_msgs::msg::OccupancyGrid current_map_;
//   bool map_received_ = false;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<VirtualScanNode>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <chrono>
#include <cmath>
#include <memory>
#include <algorithm>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class VirtualScanNode : public rclcpp::Node
{
public:
  VirtualScanNode()
  : Node("virtual_scan_node")
  {
    // 1. Subscribe to the Map (Static Reference)
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/virtual_map", map_qos, std::bind(&VirtualScanNode::map_callback, this, std::placeholders::_1));

    // 2. Subscribe to REAL SCAN (Template)
    // We use this to copy exact angles and timing (5Hz, 0.2s, 1440 samples)
    real_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&VirtualScanNode::scan_callback, this, std::placeholders::_1));

    // 3. Publisher
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/virtual_scan", 10);

    // 4. TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Virtual Scan Node (Copycat Mode) Started.");
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    map_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Map Received.");
  }

  // Triggered whenever real scan arrives (approx 5Hz)
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr real_scan)
  {
    if (!map_received_) return;

    // Get Robot Position
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      return; 
    }

    publish_virtual_scan(real_scan, t);
  }

  void publish_virtual_scan(const sensor_msgs::msg::LaserScan::SharedPtr template_scan, 
                            const geometry_msgs::msg::TransformStamped & transform)
  {
    auto scan = sensor_msgs::msg::LaserScan();
    
    // --- COPY CONFIG FROM REAL SCAN ---
    // This automatically fixes the 5Hz vs 0.2s issue and angle alignment
    scan.header.stamp = template_scan->header.stamp; 
    scan.header.frame_id = template_scan->header.frame_id; // e.g. base_scan
    
    scan.angle_min = template_scan->angle_min;
    scan.angle_max = template_scan->angle_max;
    scan.angle_increment = template_scan->angle_increment;
    scan.time_increment = template_scan->time_increment;
    scan.scan_time = template_scan->scan_time; // Will be 0.2 based on your XML
    scan.range_min = template_scan->range_min;
    scan.range_max = template_scan->range_max;
    
    // Prepare arrays
    int num_readings = template_scan->ranges.size(); // Will be 1440
    scan.ranges.assign(num_readings, std::numeric_limits<float>::infinity());

    // --- RAY CASTING SETUP ---
    double robot_x = transform.transform.translation.x;
    double robot_y = transform.transform.translation.y;
    double robot_yaw = tf2::getYaw(transform.transform.rotation);

    float resolution = current_map_.info.resolution;
    float origin_x = current_map_.info.origin.position.x;
    float origin_y = current_map_.info.origin.position.y;
    int width = current_map_.info.width;
    int height = current_map_.info.height;

    // --- RAY CAST LOOP ---
    // Optimization: Pre-calculate sin/cos if 1440 is constant, but this is fine for 5Hz
    for (int i = 0; i < num_readings; ++i) {
        double angle = scan.angle_min + i * scan.angle_increment;
        double global_angle = robot_yaw + angle;

        double ray_x = robot_x;
        double ray_y = robot_y;
        double dist = 0.0;
        double step = resolution; // Optimization: Bresenham is faster, but this is readable

        while (dist < scan.range_max) {
            dist += step;
            ray_x = robot_x + dist * cos(global_angle);
            ray_y = robot_y + dist * sin(global_angle);

            int grid_x = (int)((ray_x - origin_x) / resolution);
            int grid_y = (int)((ray_y - origin_y) / resolution);

            if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
                break;
            }

            int index = grid_y * width + grid_x;
            if (current_map_.data[index] == 100) {
                scan.ranges[i] = dist; // Hit wall
                break;
            }
        }
    }
    scan_pub_->publish(scan);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr real_scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::OccupancyGrid current_map_;
  bool map_received_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualScanNode>());
  rclcpp::shutdown();
  return 0;
}