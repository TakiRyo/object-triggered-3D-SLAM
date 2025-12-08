#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cmath>
#include <vector>
#include <mutex>
#include <unordered_map>

// Helper to hash (x, y) coordinates for the grid map
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
    this->declare_parameter("distance_threshold", 2.0); // 1 meter gap to consider "Difference"
    this->declare_parameter("time_threshold", 3.0);     // 3 seconds persistence
    this->declare_parameter("grid_resolution", 0.1);    // 10cm grid cells
    this->declare_parameter("decay_rate", 0.5);         // How fast timer drops if detection stops (sec/sec)

    this->get_parameter("distance_threshold", dist_thresh_);
    this->get_parameter("time_threshold", time_thresh_);
    this->get_parameter("grid_resolution", grid_res_);
    this->get_parameter("decay_rate", decay_rate_);

    // --- Subs & Pubs ---
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ChangeDetectorNode::scanCallback, this, std::placeholders::_1));

    virtual_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/virtual_scan", 10, std::bind(&ChangeDetectorNode::virtualScanCallback, this, std::placeholders::_1));

    // Consolidated Output: /detected_goals (Green = New, Purple = Disappeared)
    change_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detected_goals", 10);

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    last_update_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Change Detector Initialized. Time Thresh: %.1fs", time_thresh_);
  }

private:
  sensor_msgs::msg::LaserScan::SharedPtr last_virtual_scan_;
  std::mutex virtual_mutex_;
  
  // Timers for persistence
  // Key: Grid Cell (x,y), Value: Seconds existed
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

    // --- 1. Calculate DT ---
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;

    // --- 2. Get Transform (Base -> Map) ---
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("map", real_scan->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) { return; }

    // --- 3. Identify Instantaneous Changes ---
    // We use temporary sets to know which cells were "hit" this frame
    std::unordered_map<GridKey, bool, GridKeyHasher> hit_new_cells;
    std::unordered_map<GridKey, bool, GridKeyHasher> hit_gone_cells;

    for (size_t i = 0; i < real_scan->ranges.size(); ++i) {
        float r_real = real_scan->ranges[i];
        float r_virt = v_scan->ranges[i];
        
        // Skip bad data
        bool r_real_valid = !std::isnan(r_real) && !std::isinf(r_real) && r_real < real_scan->range_max;
        bool r_virt_valid = !std::isnan(r_virt) && !std::isinf(r_virt); // Infinite map means "Open Space"

        float angle = real_scan->angle_min + i * real_scan->angle_increment;

        // --- Case A: NEW OBJECT (Real < Virtual - Thresh) ---
        // (Or Real sees something where Virtual sees Infinity)
        if (r_real_valid) {
            bool is_new = false;
            if (!r_virt_valid) is_new = true; // Map is empty, Real is hit
            else if (r_real < (r_virt - dist_thresh_)) is_new = true; // Real is closer

            if (is_new) {
                // Calculate Map Coords
                geometry_msgs::msg::Point p = transformPoint(r_real, angle, t);
                GridKey k = { (int)(p.x / grid_res_), (int)(p.y / grid_res_) };
                hit_new_cells[k] = true;
            }
        }

        // --- Case B: DISAPPEARED OBJECT (Real > Virtual + Thresh) ---
        if (r_virt_valid) { // Only check if there WAS a wall
            bool is_gone = false;
            if (!r_real_valid) is_gone = true; // Real is Inf (See through)
            else if (r_real > (r_virt + dist_thresh_)) is_gone = true; // Real is farther

            if (is_gone) {
                // Calculate Map Coords OF THE OLD WALL
                geometry_msgs::msg::Point p = transformPoint(r_virt, angle, t);
                GridKey k = { (int)(p.x / grid_res_), (int)(p.y / grid_res_) };
                hit_gone_cells[k] = true;
            }
        }
    }

    // --- 4. Update Persistence Grids ---
    updateGrid(new_obj_grid_, hit_new_cells, dt);
    updateGrid(gone_obj_grid_, hit_gone_cells, dt);

    // --- 5. Publish Persistent Points ---
    publishChanges();
  }

  // Generic Grid Updater
  void updateGrid(std::unordered_map<GridKey, float, GridKeyHasher>& grid, 
                  const std::unordered_map<GridKey, bool, GridKeyHasher>& hits, 
                  double dt) 
  {
      // 1. Increment timer for hits
      for (const auto& pair : hits) {
          grid[pair.first] += dt;
          // Cap at slightly above threshold to prevent infinite buildup
          if (grid[pair.first] > (time_thresh_ * 1.5)) grid[pair.first] = time_thresh_ * 1.5;
      }

      // 2. Decay timer for misses
      auto it = grid.begin();
      while (it != grid.end()) {
          if (hits.find(it->first) == hits.end()) {
              it->second -= (decay_rate_ * dt);
          }
          
          if (it->second <= 0.0) {
              it = grid.erase(it); // Remove empty cells to save memory
          } else {
              ++it;
          }
      }
  }

  void publishChanges() {
      auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
      cloud->header.frame_id = "map";
      cloud->header.stamp = this->now();
      cloud->height = 1;
      
      sensor_msgs::PointCloud2Modifier modifier(*cloud);
      modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

      // Count points first
      int count = 0;
      for (const auto& p : new_obj_grid_) if (p.second > time_thresh_) count++;
      for (const auto& p : gone_obj_grid_) if (p.second > time_thresh_) count++;
      
      modifier.resize(count);

      sensor_msgs::PointCloud2Iterator<float> out_x(*cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> out_y(*cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> out_z(*cloud, "z");
      sensor_msgs::PointCloud2Iterator<uint8_t> out_r(*cloud, "r");
      sensor_msgs::PointCloud2Iterator<uint8_t> out_g(*cloud, "g");
      sensor_msgs::PointCloud2Iterator<uint8_t> out_b(*cloud, "b");

      // Helper to add point
      auto addPoint = [&](const GridKey& k, uint8_t r, uint8_t g, uint8_t b) {
          *out_x = (k.x * grid_res_) + (grid_res_/2.0); // Center of cell
          *out_y = (k.y * grid_res_) + (grid_res_/2.0);
          *out_z = 0.0;
          *out_r = r; *out_g = g; *out_b = b;
          ++out_x; ++out_y; ++out_z; ++out_r; ++out_g; ++out_b;
      };

      // Green for New, Purple for Gone
      for (const auto& p : new_obj_grid_) {
          if (p.second > time_thresh_) addPoint(p.first, 0, 255, 0); 
      }
      for (const auto& p : gone_obj_grid_) {
          if (p.second > time_thresh_) addPoint(p.first, 255, 0, 255);
      }

      change_pub_->publish(std::move(cloud));
  }

  geometry_msgs::msg::Point transformPoint(float r, float angle, const geometry_msgs::msg::TransformStamped& t) {
      // Local Coords
      float lx = r * std::cos(angle);
      float ly = r * std::sin(angle);

      // Manual Transform
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr change_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChangeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}