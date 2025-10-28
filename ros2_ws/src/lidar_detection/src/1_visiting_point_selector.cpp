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
  geometry_msgs::msg::Pose pose;  // in map frame
  double distance_to_robot;
};

class VisitingPointSelector : public rclcpp::Node {
public:
  VisitingPointSelector()
  : Node("visiting_point_selector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {

    // params
    declare_parameter<double>("offset_from_object", 1);       // m away from box surface
    declare_parameter<bool>("face_object", true);                // orient towards object center
    declare_parameter<std::string>("marker_array_topic", "/object_markers");
    declare_parameter<std::string>("single_marker_topic", "/visualization_marker");

    offset_      = get_parameter("offset_from_object").as_double();
    face_object_ = get_parameter("face_object").as_bool();
    topic_array_ = get_parameter("marker_array_topic").as_string();
    topic_single_= get_parameter("single_marker_topic").as_string();

    // subs
    sub_array_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      topic_array_, 10,
      std::bind(&VisitingPointSelector::onMarkers, this, std::placeholders::_1));

    // optional: some users only have single-marker stream; we accept it too
    sub_single_ = create_subscription<visualization_msgs::msg::Marker>(
      topic_single_, 10,
      std::bind(&VisitingPointSelector::onSingleMarker, this, std::placeholders::_1));

    // pubs
    goal_pub_   = create_publisher<geometry_msgs::msg::PoseStamped>("/target_goal", 10);
    viz_pub_    = create_publisher<visualization_msgs::msg::Marker>("/visiting_point_marker", 10);

    RCLCPP_INFO(get_logger(), "✅ visiting_point_selector up. offset=%.2f m, face_object=%s",
                offset_, face_object_ ? "true" : "false");
  }

private:
  // ------------ callbacks ------------
  void onMarkers(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    if (msg->markers.empty()) return;
    processMarkers(msg->markers);
  }

  void onSingleMarker(const visualization_msgs::msg::Marker::SharedPtr msg) {
    // treat it like an array of one
    processMarkers(std::vector<visualization_msgs::msg::Marker>{*msg});
  }

  // ------------ core ------------
  void processMarkers(const std::vector<visualization_msgs::msg::Marker>& markers) {
    geometry_msgs::msg::Point robot_xy;
    if (!getRobotXY(robot_xy)) return;

    // build all candidate visiting points from all markers, choose nearest
    VisitingCandidate best;
    best.distance_to_robot = std::numeric_limits<double>::infinity();
    bool have_candidate = false;

    for (const auto& m : markers) {
      if (m.header.frame_id.empty()) continue;
      if (m.type != visualization_msgs::msg::Marker::CUBE) continue;  // expecting boxes

      // marker center & extents (assumed axis-aligned in map)
      const double cx = m.pose.position.x;
      const double cy = m.pose.position.y;
      const double ex = std::max(0.05, (double)m.scale.x) / 2.0;  // half-lengths
      const double ey = std::max(0.05, (double)m.scale.y) / 2.0;

      // 4 edge points (front/back/left/right) with offset from surface
      std::vector<geometry_msgs::msg::Pose> candidates;

      candidates.push_back(makePose(cx + (ex + offset_), cy, cx, cy)); // +x side
      candidates.push_back(makePose(cx - (ex + offset_), cy, cx, cy)); // -x side
      candidates.push_back(makePose(cx, cy + (ey + offset_), cx, cy)); // +y side
      candidates.push_back(makePose(cx, cy - (ey + offset_), cx, cy)); // -y side

      // choose nearest to robot among these 4
      for (auto& c : candidates) {
        const double dx = c.position.x - robot_xy.x;
        const double dy = c.position.y - robot_xy.y;
        const double d  = std::hypot(dx, dy);
        if (d < best.distance_to_robot) {
          best.pose = c;
          best.distance_to_robot = d;
          have_candidate = true;
        }
      }
    }

    if (!have_candidate) return;

    // publish goal + a small marker at that visiting point
    publishGoal(best.pose);
    publishVisMarker(best.pose);
  }

  geometry_msgs::msg::Pose makePose(double x, double y, double look_at_x, double look_at_y) const {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = 0.0;

    double yaw = 0.0;
    if (face_object_) {
      // orientation that faces the object center
      yaw = std::atan2(look_at_y - y, look_at_x - x);
    }
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    p.orientation = tf2::toMsg(q);
    return p;
  }

  bool getRobotXY(geometry_msgs::msg::Point& out) {
    try {
      auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      out.x = tf.transform.translation.x;
      out.y = tf.transform.translation.y;
      out.z = 0.0;
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF not ready: %s", ex.what());
      return false;
    }
  }

  void publishGoal(const geometry_msgs::msg::Pose& pose_map) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = now();
    goal.pose = pose_map;
    goal_pub_->publish(goal);

    RCLCPP_INFO(get_logger(), "🎯 goal → (%.2f, %.2f) [d yaw]",
                goal.pose.position.x, goal.pose.position.y);
  }

  void publishVisMarker(const geometry_msgs::msg::Pose& pose_map) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now();
    m.ns = "visiting_point";
    m.id = 0;  // we reuse single id so it updates, not accumulate
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = pose_map;
    m.scale.x = 0.2;
    m.scale.y = 0.2;
    m.scale.z = 0.2;
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 0.9f;
    viz_pub_->publish(m);
  }

  // members
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_array_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr       sub_single_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr          goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr          viz_pub_;

  double offset_;
  bool   face_object_;
  std::string topic_array_;
  std::string topic_single_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisitingPointSelector>());
  rclcpp::shutdown();
  return 0;
}
