#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <random>
#include <vector>

struct Line {
  float a, b, c; // ax + by + c = 0
  std::vector<std::pair<float, float>> inliers;
};

class RoomShapeEstimator : public rclcpp::Node {
public:
  RoomShapeEstimator()
      : Node("room_shape_estimator") {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&RoomShapeEstimator::scanCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/room_shape", 10);

    RCLCPP_INFO(this->get_logger(), "✅ RoomShapeEstimator started (RANSAC-based wall detection)");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // ------------------- helper: distance from point to line -------------------
  float pointLineDist(const Line &line, float x, float y) {
    return std::fabs(line.a * x + line.b * y + line.c) / std::sqrt(line.a * line.a + line.b * line.b);
  }

  // ------------------- main RANSAC loop -------------------
  Line ransacLine(const std::vector<std::pair<float, float>> &points, int iterations = 500, float threshold = 0.05) {
    Line best_line;
    int best_inliers = 0;
    std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<> dist(0, points.size() - 1);

    for (int i = 0; i < iterations; ++i) {
      int i1 = dist(gen);
      int i2 = dist(gen);
      if (i1 == i2) continue;

      auto p1 = points[i1];
      auto p2 = points[i2];

      // line: ax + by + c = 0
      float a = p1.second - p2.second;
      float b = p2.first - p1.first;
      float c = p1.first * p2.second - p2.first * p1.second;
      float norm = std::sqrt(a * a + b * b);
      if (norm < 1e-6) continue;
      a /= norm; b /= norm; c /= norm;

      Line line = {a, b, c, {}};
      int inliers = 0;
      for (auto &p : points) {
        if (std::fabs(a * p.first + b * p.second + c) < threshold) {
          line.inliers.push_back(p);
          inliers++;
        }
      }

      if (inliers > best_inliers) {
        best_inliers = inliers;
        best_line = line;
      }
    }
    return best_line;
  }

  // ------------------- visualization helper -------------------
  void publishLines(const std::vector<Line> &lines) {
    visualization_msgs::msg::MarkerArray ma;
    int id = 0;

    for (auto &line : lines) {
      if (line.inliers.empty()) continue;

      // find endpoints (min/max projection)
      float min_proj = 1e6, max_proj = -1e6;
      float vx = -line.b, vy = line.a; // direction vector
      for (auto &p : line.inliers) {
        float proj = p.first * vx + p.second * vy;
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
      }

      float midx = 0, midy = 0;
      for (auto &p : line.inliers) { midx += p.first; midy += p.second; }
      midx /= line.inliers.size(); midy /= line.inliers.size();

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = this->now();
      marker.ns = "walls";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.scale.x = 0.05;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      geometry_msgs::msg::Point p1, p2;
      p1.x = midx + vx * min_proj;
      p1.y = midy + vy * min_proj;
      p2.x = midx + vx * max_proj;
      p2.y = midy + vy * max_proj;
      marker.points.push_back(p1);
      marker.points.push_back(p2);

      ma.markers.push_back(marker);
    }

    marker_pub_->publish(ma);
  }

  // ------------------- main callback -------------------
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<std::pair<float, float>> points;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float r = msg->ranges[i];
      if (std::isnan(r) || std::isinf(r)) continue;
      float angle = msg->angle_min + i * msg->angle_increment;
      float x = r * std::cos(angle);
      float y = r * std::sin(angle);
      points.push_back({x, y});
    }

    if (points.size() < 50) return;

    std::vector<Line> walls;
    std::vector<std::pair<float, float>> remaining = points;

    for (int w = 0; w < 4; ++w) {
      Line line = ransacLine(remaining);
      if (line.inliers.size() < 30) break; // too small -> stop
      walls.push_back(line);

      // remove inliers from remaining points
      std::vector<std::pair<float, float>> new_pts;
      for (auto &p : remaining) {
        if (std::fabs(line.a * p.first + line.b * p.second + line.c) > 0.1)
          new_pts.push_back(p);
      }
      remaining = new_pts;
    }

    publishLines(walls);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoomShapeEstimator>());
  rclcpp::shutdown();
  return 0;
}
