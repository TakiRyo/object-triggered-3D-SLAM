#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <random>
#include <vector>
#include <utility>

struct Line {
  float a, b, c; // ax + by + c = 0
  std::vector<std::pair<float, float>> inliers;
};

class RoomShapeEstimator : public rclcpp::Node {
public:
  RoomShapeEstimator() : Node("room_shape_estimator") {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&RoomShapeEstimator::scanCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/room_shape", 10);

    RCLCPP_INFO(this->get_logger(), "✅ RoomShapeEstimator started (RANSAC + Room polygon)");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // ---------------- distance from point to line ----------------
  float pointLineDist(const Line &line, float x, float y) {
    return std::fabs(line.a * x + line.b * y + line.c) /
           std::sqrt(line.a * line.a + line.b * line.b);
  }

  // ---------------- intersection between 2 lines ----------------
  bool intersect(const Line &l1, const Line &l2, float &x, float &y) {
    float det = l1.a * l2.b - l2.a * l1.b;
    if (std::fabs(det) < 1e-6) return false; // parallel
    x = (l2.b * (-l1.c) - l1.b * (-l2.c)) / det;
    y = (l1.a * (-l2.c) - l2.a * (-l1.c)) / det;
    return true;
  }

  // ---------------- main RANSAC loop ----------------
  Line ransacLine(const std::vector<std::pair<float, float>> &points,
                  int iterations = 400, float threshold = 0.05) {
    Line best_line;
    int best_inliers = 0;
    std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<> dist(0, points.size() - 1);

    for (int i = 0; i < iterations; ++i) {
      int i1 = dist(gen), i2 = dist(gen);
      if (i1 == i2) continue;
      auto p1 = points[i1], p2 = points[i2];

      // line equation ax + by + c = 0
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

  // ---------------- publish visualization markers ----------------
  void publishMarkers(const std::vector<Line> &walls,
                      const std::vector<std::pair<float, float>> &corners) {
    visualization_msgs::msg::MarkerArray ma;
    int id = 0;

    // ---- Draw walls ----
    for (auto &line : walls) {
      if (line.inliers.empty()) continue;

      float min_proj = 1e6, max_proj = -1e6;
      float vx = -line.b, vy = line.a; // direction vector

      for (auto &p : line.inliers) {
        float proj = p.first * vx + p.second * vy;
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
      }

      float midx = 0, midy = 0;
      for (auto &p : line.inliers) { midx += p.first; midy += p.second; }
      midx /= line.inliers.size();
      midy /= line.inliers.size();

      // extend a bit to make walls continuous
      float ext = 1.0;
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "base_link";
      m.header.stamp = rclcpp::Time(0);
      m.ns = "walls";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.scale.x = 0.05;
      m.color.r = 1.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.color.a = 1.0;

      geometry_msgs::msg::Point p1, p2;
      p1.x = midx + vx * (min_proj - ext);
      p1.y = midy + vy * (min_proj - ext);
      p1.z = 0.0;
      p2.x = midx + vx * (max_proj + ext);
      p2.y = midy + vy * (max_proj + ext);
      p2.z = 0.0;
      m.points.push_back(p1);
      m.points.push_back(p2);
      ma.markers.push_back(m);
    }

    // ---- Draw corners ----
    visualization_msgs::msg::Marker corner_marker;
    corner_marker.header.frame_id = "base_link";
    corner_marker.header.stamp = rclcpp::Time(0);
    corner_marker.ns = "corners";
    corner_marker.id = id++;
    corner_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    corner_marker.scale.x = 0.15;
    corner_marker.scale.y = 0.15;
    corner_marker.scale.z = 0.15;
    corner_marker.color.r = 0.0;
    corner_marker.color.g = 1.0;
    corner_marker.color.b = 0.0;
    corner_marker.color.a = 1.0;

    for (auto &p : corners) {
      geometry_msgs::msg::Point pt;
      pt.x = p.first;
      pt.y = p.second;
      pt.z = 0.0;
      corner_marker.points.push_back(pt);
    }
    ma.markers.push_back(corner_marker);

    // ---- Draw room polygon ----
    if (corners.size() >= 3) {
      visualization_msgs::msg::Marker poly;
      poly.header.frame_id = "base_link";
      poly.header.stamp = rclcpp::Time(0);
      poly.ns = "room_polygon";
      poly.id = id++;
      poly.type = visualization_msgs::msg::Marker::LINE_STRIP;
      poly.scale.x = 0.07;
      poly.color.r = 0.0;
      poly.color.g = 0.5;
      poly.color.b = 1.0;
      poly.color.a = 1.0;

      for (auto &p : corners) {
        geometry_msgs::msg::Point pt;
        pt.x = p.first;
        pt.y = p.second;
        pt.z = 0.0;
        poly.points.push_back(pt);
      }

      // close polygon
      geometry_msgs::msg::Point first_pt;
      first_pt.x = corners[0].first;
      first_pt.y = corners[0].second;
      first_pt.z = 0.0;
      poly.points.push_back(first_pt);

      ma.markers.push_back(poly);
    }

    marker_pub_->publish(ma);
  }

  // ---------------- main callback ----------------
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

    // --- RANSAC wall detection ---
    std::vector<Line> walls;
    std::vector<std::pair<float, float>> remaining = points;

    for (int w = 0; w < 4; ++w) {
      Line line = ransacLine(remaining);
      if (line.inliers.size() < 25) break;
      walls.push_back(line);

      // remove inliers
      std::vector<std::pair<float, float>> new_pts;
      for (auto &p : remaining) {
        if (std::fabs(line.a * p.first + line.b * p.second + line.c) > 0.1)
          new_pts.push_back(p);
      }
      remaining = new_pts;
    }

    // --- Find intersections (corners) ---
    std::vector<std::pair<float, float>> corners;
    for (size_t i = 0; i < walls.size(); ++i) {
      float x, y;
      size_t j = (i + 1) % walls.size();
      if (intersect(walls[i], walls[j], x, y)) {
        corners.push_back({x, y});
      }
    }

    publishMarkers(walls, corners);
  }
};

// ---------- main ----------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoomShapeEstimator>());
  rclcpp::shutdown();
  return 0;
}
