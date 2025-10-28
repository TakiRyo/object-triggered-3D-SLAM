#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <vector>
#include <limits>

struct VisitingCandidate {
  geometry_msgs::msg::Pose pose;
  double distance_to_robot;
};

class VisitingPointSelector : public rclcpp::Node {
public:
  VisitingPointSelector()
  : Node("visiting_point_selector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {

    declare_parameter<double>("offset_from_object", 0.6);
    offset_ = get_parameter("offset_from_object").as_double();

    sub_markers_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/object_markers", 10,
      std::bind(&VisitingPointSelector::onMarkers, this, std::placeholders::_1));

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/target_goal", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visiting_markers", 10);

    RCLCPP_INFO(get_logger(), "✅ VisitingPointSelector started (multi-marker version)");
  }

private:
  void onMarkers(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    if (msg->markers.empty()) return;

    geometry_msgs::msg::Point robot_xy;
    if (!getRobotPosition(robot_xy)) return;

    visualization_msgs::msg::MarkerArray visiting_markers;
    std::vector<VisitingCandidate> candidates;

    int id = 0;

    for (const auto &m : msg->markers) {
      if (m.type != visualization_msgs::msg::Marker::CUBE) continue;

      double cx = m.pose.position.x;
      double cy = m.pose.position.y;
      double ex = m.scale.x / 2.0;
      double ey = m.scale.y / 2.0;

      // 4 edge points
      std::vector<geometry_msgs::msg::Pose> edge_points = {
        makePose(cx + (ex + offset_), cy, cx, cy),
        makePose(cx - (ex + offset_), cy, cx, cy),
        makePose(cx, cy + (ey + offset_), cx, cy),
        makePose(cx, cy - (ey + offset_), cx, cy)
      };

      // Choose nearest edge to robot for this marker
      VisitingCandidate best;
      best.distance_to_robot = std::numeric_limits<double>::infinity();

      for (auto &p : edge_points) {
        double dx = p.position.x - robot_xy.x;
        double dy = p.position.y - robot_xy.y;
        double dist = std::hypot(dx, dy);
        if (dist < best.distance_to_robot) {
          best.pose = p;
          best.distance_to_robot = dist;
        }
      }

      // Save this visiting point
      candidates.push_back(best);

      // Visualization marker (green sphere)
      visualization_msgs::msg::Marker vm;
      vm.header.frame_id = "map";
      vm.header.stamp = this->now();
      vm.ns = "visiting_points";
      vm.id = id++;
      vm.type = visualization_msgs::msg::Marker::SPHERE;
      vm.action = visualization_msgs::msg::Marker::ADD;
      vm.pose = best.pose;
      vm.scale.x = 0.2;
      vm.scale.y = 0.2;
      vm.scale.z = 0.2;
      vm.color.r = 0.0f;
      vm.color.g = 1.0f;
      vm.color.b = 0.0f;
      vm.color.a = 0.9f;
      visiting_markers.markers.push_back(vm);
    }

    // Publish all visiting markers
    marker_pub_->publish(visiting_markers);

    // Publish nearest one as goal
    if (!candidates.empty()) {
      auto nearest = *std::min_element(
        candidates.begin(), candidates.end(),
        [](const auto &a, const auto &b) { return a.distance_to_robot < b.distance_to_robot; });

      publishGoal(nearest.pose);
    }
  }

  bool getRobotPosition(geometry_msgs::msg::Point &pos) {
    try {
      auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      pos.x = tf.transform.translation.x;
      pos.y = tf.transform.translation.y;
      pos.z = 0.0;
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF not ready: %s", ex.what());
      return false;
    }
  }

  geometry_msgs::msg::Pose makePose(double x, double y, double look_x, double look_y) {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = 0.0;

    double yaw = std::atan2(look_y - y, look_x - x);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    p.orientation = tf2::toMsg(q);
    return p;
  }

  void publishGoal(const geometry_msgs::msg::Pose &pose_map) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose = pose_map;
    goal_pub_->publish(goal);
    RCLCPP_INFO(this->get_logger(), "🎯 Published target goal: (x=%.2f, y=%.2f)",
                pose_map.position.x, pose_map.position.y);
  }

  // members
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_markers_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double offset_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisitingPointSelector>());
  rclcpp::shutdown();
  return 0;
}
