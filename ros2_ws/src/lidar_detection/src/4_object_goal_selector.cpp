#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

struct TrackedCluster {
  geometry_msgs::msg::Point first_position;
  geometry_msgs::msg::Point last_position;
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
  double robot_motion_at_start;
  bool confirmed;
};

class ObjectGoalSelector : public rclcpp::Node
{
public:
  ObjectGoalSelector()
  : Node("object_goal_selector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    object_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/object_clusters", 10,
      std::bind(&ObjectGoalSelector::cloudCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

    // Parameters
    visible_time_threshold_   = 3.0;   // seconds
    moved_distance_threshold_ = 0.0;   // robot motion
    centroid_drift_tolerance_ = 3.0;   // meters
    merge_threshold_          = 0.5;   // cluster association distance

    total_robot_displacement_ = 0.0;
    last_robot_x_ = 0.0;
    last_robot_y_ = 0.0;

    marker_id_ = 0;

    RCLCPP_INFO(this->get_logger(), "✅ ObjectGoalSelector started (3s + 2m + 1m tolerance)");
  }

private:
  // --- TF helper: get robot position in map frame ---
  geometry_msgs::msg::Point getRobotPosition()
  {
    geometry_msgs::msg::Point pos;
    try {
      auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      pos.x = tf.transform.translation.x;
      pos.y = tf.transform.translation.y;
      pos.z = 0.0;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF not ready: %s", ex.what());
    }
    return pos;
  }

  // --- Callback for object clusters ---
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    geometry_msgs::msg::Point robot_pos = getRobotPosition();
    double dx = robot_pos.x - last_robot_x_;
    double dy = robot_pos.y - last_robot_y_;
    total_robot_displacement_ += std::sqrt(dx*dx + dy*dy);
    last_robot_x_ = robot_pos.x;
    last_robot_y_ = robot_pos.y;

    // compute centroid of all points
    geometry_msgs::msg::Point centroid = computeCentroid(msg);
    if (std::isnan(centroid.x) || std::isnan(centroid.y)) return;

    updateClusters(centroid);
  }

  // --- Compute centroid of cloud ---
  geometry_msgs::msg::Point computeCentroid(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    geometry_msgs::msg::Point c;
    c.x = c.y = c.z = 0.0;
    size_t count = 0;

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      c.x += *iter_x;
      c.y += *iter_y;
      c.z += *iter_z;
      count++;
    }

    if (count == 0) {
      c.x = c.y = c.z = std::numeric_limits<float>::quiet_NaN();
      return c;
    }

    c.x /= count;
    c.y /= count;
    c.z /= count;
    return c;
  }

  // --- Cluster tracking logic ---
  void updateClusters(const geometry_msgs::msg::Point &centroid)
  {
    rclcpp::Time now = this->now();
    bool matched = false;

    for (auto &cl : clusters_)
    {
      double dx = centroid.x - cl.last_position.x;
      double dy = centroid.y - cl.last_position.y;
      double dist = std::sqrt(dx*dx + dy*dy);

      if (dist < merge_threshold_)
      {
        cl.last_seen = now;
        cl.last_position = centroid;

        double visible_time = (cl.last_seen - cl.first_seen).seconds();
        double robot_moved = total_robot_displacement_ - cl.robot_motion_at_start;

        double drift = std::hypot(cl.last_position.x - cl.first_position.x,
                                  cl.last_position.y - cl.first_position.y);

        if (!cl.confirmed &&
            visible_time >= visible_time_threshold_ &&
            robot_moved >= moved_distance_threshold_ &&
            drift <= centroid_drift_tolerance_)
        {
          cl.confirmed = true;
          publishMarker(cl.last_position);
          RCLCPP_INFO(this->get_logger(),
                      "🎯 Confirmed stable cluster: visible %.1fs, robot moved %.1fm, drift %.2fm",
                      visible_time, robot_moved, drift);
        }

        matched = true;
        break;
      }
    }

    if (!matched)
    {
      TrackedCluster new_cl;
      new_cl.first_position = centroid;
      new_cl.last_position  = centroid;
      new_cl.first_seen = now;
      new_cl.last_seen = now;
      new_cl.robot_motion_at_start = total_robot_displacement_;
      new_cl.confirmed = false;
      clusters_.push_back(new_cl);
    }
  }

  // --- Publish red marker ---
  void publishMarker(const geometry_msgs::msg::Point &pt)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "confirmed_clusters";
    marker.id = marker_id_++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = pt;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7f;

    marker.lifetime = rclcpp::Duration::from_seconds(0); // persistent
    marker_pub_->publish(marker);

    RCLCPP_INFO(this->get_logger(), "📍 Marker placed at (%.2f, %.2f)", pt.x, pt.y);
  }

  // --- Members ---
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr object_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<TrackedCluster> clusters_;

  double visible_time_threshold_;
  double moved_distance_threshold_;
  double centroid_drift_tolerance_;
  double merge_threshold_;

  double last_robot_x_, last_robot_y_;
  double total_robot_displacement_;
  int marker_id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectGoalSelector>());
  rclcpp::shutdown();
  return 0;
}
