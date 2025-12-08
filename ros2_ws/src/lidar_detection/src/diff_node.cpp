#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <mutex>
#include <unordered_map>
#include <algorithm>

struct GridKey {
    int x, y;
    bool operator==(const GridKey &other) const { return x == other.x && y == other.y; }
};
struct GridKeyHasher {
    std::size_t operator()(const GridKey &k) const {
        return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1);
    }
};

class ChangeDetectorNode : public rclcpp::Node
{
public:
  ChangeDetectorNode() : Node("change_detector_node")
  {
    // --- Parameters ---
    this->declare_parameter("distance_threshold", 0.5); 
    this->declare_parameter("time_threshold", 2.0);
    this->declare_parameter("grid_resolution", 0.1); 
    this->declare_parameter("decay_rate", 0.5);         

    this->get_parameter("distance_threshold", dist_thresh_);
    this->get_parameter("time_threshold", time_thresh_);
    this->get_parameter("grid_resolution", grid_res_);
    this->get_parameter("decay_rate", decay_rate_);
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ChangeDetectorNode::scanCallback, this, std::placeholders::_1));

    virtual_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/virtual_scan", 10, std::bind(&ChangeDetectorNode::virtualScanCallback, this, std::placeholders::_1));

    // --- TWO SEPARATE PUBLISHERS ---
    new_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/added_objects", 10);
    removed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/removed_objects", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    last_update_time_ = this->now();
  }

private:
  sensor_msgs::msg::LaserScan::SharedPtr last_virtual_scan_;
  std::mutex virtual_mutex_;
  
  std::unordered_map<GridKey, float, GridKeyHasher> new_obj_grid_;
  std::unordered_map<GridKey, float, GridKeyHasher> gone_obj_grid_;

  double dist_thresh_;
  double time_thresh_;
  double grid_res_;
  double decay_rate_;
  rclcpp::Time last_update_time_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void virtualScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(virtual_mutex_);
    last_virtual_scan_ = msg;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr real_scan)
  {
    sensor_msgs::msg::LaserScan::SharedPtr v_scan;
    {
        std::lock_guard<std::mutex> lock(virtual_mutex_);
        if(!last_virtual_scan_) return;
        v_scan = last_virtual_scan_;
    }
    if (real_scan->ranges.size() != v_scan->ranges.size()) return;

    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;

    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("map", real_scan->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) { return; }

    std::unordered_map<GridKey, bool, GridKeyHasher> hit_new_cells;
    std::unordered_map<GridKey, bool, GridKeyHasher> hit_gone_cells;

    int scan_size = real_scan->ranges.size();
    
    // --- 1. DETECT NEW ---
    for (int i = 0; i < scan_size; ++i) {
        float r_real = real_scan->ranges[i];
        if (std::isnan(r_real) || std::isinf(r_real) || r_real > real_scan->range_max) continue;

        float angle = real_scan->angle_min + i * real_scan->angle_increment;
        float rx = r_real * std::cos(angle);
        float ry = r_real * std::sin(angle);

        bool near_wall = false;
        int search_window = 20; 
        for (int j = std::max(0, i - search_window); j < std::min(scan_size, i + search_window); ++j) {
            float r_virt = v_scan->ranges[j];
            if (std::isinf(r_virt)) continue;
            float v_angle = v_scan->angle_min + j * v_scan->angle_increment;
            float vx = r_virt * std::cos(v_angle);
            float vy = r_virt * std::sin(v_angle);
            if (std::hypot(rx - vx, ry - vy) < dist_thresh_) {
                near_wall = true;
                break; 
            }
        }
        if (!near_wall) {
            geometry_msgs::msg::Point p = transformPoint(r_real, angle, t);
            GridKey k = { (int)(p.x / grid_res_), (int)(p.y / grid_res_) };
            hit_new_cells[k] = true;
        }
    }

    // --- 2. DETECT REMOVED ---
    for (int i = 0; i < scan_size; ++i) {
        float r_virt = v_scan->ranges[i];
        if (std::isinf(r_virt) || std::isnan(r_virt)) continue;

        float angle = v_scan->angle_min + i * v_scan->angle_increment;
        float vx = r_virt * std::cos(angle);
        float vy = r_virt * std::sin(angle);

        bool wall_still_exists = false;
        int search_window = 20;
        for (int j = std::max(0, i - search_window); j < std::min(scan_size, i + search_window); ++j) {
            float r_real = real_scan->ranges[j];
            if (std::isinf(r_real) || r_real > real_scan->range_max) continue;
            float r_angle = real_scan->angle_min + j * real_scan->angle_increment;
            float rx = r_real * std::cos(r_angle);
            float ry = r_real * std::sin(r_angle);
            if (std::hypot(vx - rx, vy - ry) < dist_thresh_) {
                wall_still_exists = true;
                break; 
            }
        }
        if (!wall_still_exists) {
            geometry_msgs::msg::Point p = transformPoint(r_virt, angle, t);
            GridKey k = { (int)(p.x / grid_res_), (int)(p.y / grid_res_) };
            hit_gone_cells[k] = true;
        }
    }

    // --- 3. Update & Publish ---
    updateGrid(new_obj_grid_, hit_new_cells, dt);
    updateGrid(gone_obj_grid_, hit_gone_cells, dt);
    
    // Publish separate topics
    publishCloud(new_obj_grid_, new_pub_, 0, 255, 0);       // GREEN -> New
    publishCloud(gone_obj_grid_, removed_pub_, 255, 0, 255); // PURPLE -> Removed
  }

  void updateGrid(std::unordered_map<GridKey, float, GridKeyHasher>& grid, 
                  const std::unordered_map<GridKey, bool, GridKeyHasher>& hits, 
                  double dt) 
  {
      for (const auto& pair : hits) {
          grid[pair.first] += dt;
          if (grid[pair.first] > (time_thresh_ * 1.5)) grid[pair.first] = time_thresh_ * 1.5;
      }
      auto it = grid.begin();
      while (it != grid.end()) {
          if (hits.find(it->first) == hits.end()) {
              it->second -= (decay_rate_ * dt);
          }
          if (it->second <= 0.0) it = grid.erase(it);
          else ++it;
      }
  }

  // Generic helper to publish a cloud from a grid
  void publishCloud(const std::unordered_map<GridKey, float, GridKeyHasher>& grid,
                    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                    uint8_t r, uint8_t g, uint8_t b) 
  {
      auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
      cloud->header.frame_id = "map";
      cloud->header.stamp = this->now();
      cloud->height = 1;
      
      sensor_msgs::PointCloud2Modifier modifier(*cloud);
      modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

      int count = 0;
      for (const auto& p : grid) if (p.second > time_thresh_) count++;
      modifier.resize(count);

      sensor_msgs::PointCloud2Iterator<float> out_x(*cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> out_y(*cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> out_z(*cloud, "z");
      sensor_msgs::PointCloud2Iterator<uint8_t> out_r(*cloud, "r");
      sensor_msgs::PointCloud2Iterator<uint8_t> out_g(*cloud, "g");
      sensor_msgs::PointCloud2Iterator<uint8_t> out_b(*cloud, "b");

      for (const auto& p : grid) {
          if (p.second > time_thresh_) {
              *out_x = (p.first.x * grid_res_) + (grid_res_/2.0);
              *out_y = (p.first.y * grid_res_) + (grid_res_/2.0);
              *out_z = 0.0;
              *out_r = r; *out_g = g; *out_b = b;
              ++out_x; ++out_y; ++out_z; ++out_r; ++out_g; ++out_b;
          }
      }
      pub->publish(std::move(cloud));
  }

  geometry_msgs::msg::Point transformPoint(float r, float angle, const geometry_msgs::msg::TransformStamped& t) {
      float lx = r * std::cos(angle);
      float ly = r * std::sin(angle);
      double qx = t.transform.rotation.x;
      double qy = t.transform.rotation.y;
      double qz = t.transform.rotation.z;
      double qw = t.transform.rotation.w;
      double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      geometry_msgs::msg::Point p;
      p.x = t.transform.translation.x + (lx * std::cos(yaw) - ly * std::sin(yaw));
      p.y = t.transform.translation.y + (lx * std::sin(yaw) + ly * std::cos(yaw));
      return p;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr virtual_scan_sub_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr new_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr removed_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChangeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}