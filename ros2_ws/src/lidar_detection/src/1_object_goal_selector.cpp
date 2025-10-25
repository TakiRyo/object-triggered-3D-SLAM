#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

struct TrackedObject {
  geometry_msgs::msg::Point position;
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
  double robot_motion;
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
    // Subscribe to detected object clusters
    object_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/object_clusters", 10,
      std::bind(&ObjectGoalSelector::cloudCallback, this, std::placeholders::_1));

    // Publish goal positions for Nav2GoalSender
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_goal", 10);

    // Parameters
    visible_time_threshold_ = 3.0;   // seconds
    moved_distance_threshold_ = 2.0; // meters
    merge_threshold_ = 0.5;          // cluster matching
    novelty_threshold_ = 0.7;        // skip close goals

    last_robot_x_ = 0.0;
    last_robot_y_ = 0.0;
    total_robot_displacement_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "✅ ObjectGoalSelector started (3 s + 2 m + novelty check)");
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
      pos.z = tf.transform.translation.z;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF not ready: %s", ex.what());
    }
    return pos;
  }

  // --- Callback for object cluster cloud ---
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 1️⃣ Get robot motion
    geometry_msgs::msg::Point robot_pos = getRobotPosition();
    double dx = robot_pos.x - last_robot_x_;
    double dy = robot_pos.y - last_robot_y_;
    total_robot_displacement_ += std::sqrt(dx*dx + dy*dy);
    last_robot_x_ = robot_pos.x;
    last_robot_y_ = robot_pos.y;

    // 2️⃣ Extract centroids from point cloud
    std::vector<geometry_msgs::msg::Point> detections;
    extractCentroids(msg, detections);

    // 3️⃣ Update tracked objects
    updateTrackedObjects(detections);
  }

  // --- Helper: compute centroid from PointCloud2 ---
  void extractCentroids(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                        std::vector<geometry_msgs::msg::Point> &centroids)
  {
    if (msg->width == 0) return;

    // Since /object_clusters contains many small clusters, we can approximate each as one centroid
    // (for now we treat all points as a single set of possible centroids).
    geometry_msgs::msg::Point avg;
    avg.x = avg.y = avg.z = 0.0;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    size_t count = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      avg.x += *iter_x;
      avg.y += *iter_y;
      avg.z += *iter_z;
      count++;
    }

    if (count > 0) {
      avg.x /= count;
      avg.y /= count;
      avg.z /= count;
      centroids.push_back(avg);
    }
  }

  // --- Main tracking logic ---
  void updateTrackedObjects(const std::vector<geometry_msgs::msg::Point> &detections)
  {
    rclcpp::Time now = this->now();

    for (const auto &det : detections)
    {
      bool matched = false;
      for (auto &obj : tracked_objects_)
      {
        double dx = det.x - obj.position.x;
        double dy = det.y - obj.position.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist < merge_threshold_) {
          obj.last_seen = now;
          obj.position = det;
          obj.robot_motion = total_robot_displacement_;
          matched = true;

          double visible_time = (obj.last_seen - obj.first_seen).seconds();

          if (!obj.confirmed &&
              visible_time > visible_time_threshold_ &&
              obj.robot_motion > moved_distance_threshold_)
          {
            obj.confirmed = true;

            // --- Novelty check ---
            bool is_new = true;
            for (const auto &v : visited_goals_) {
              double ddx = v.x - obj.position.x;
              double ddy = v.y - obj.position.y;
              if (std::sqrt(ddx*ddx + ddy*ddy) < novelty_threshold_) {
                is_new = false;
                break;
              }
            }

            if (is_new) {
              visited_goals_.push_back(obj.position);
              publishGoal(obj.position);
            }
          }
          break;
        }
      }

      if (!matched) {
        TrackedObject new_obj;
        new_obj.position = det;
        new_obj.first_seen = now;
        new_obj.last_seen = now;
        new_obj.robot_motion = total_robot_displacement_;
        new_obj.confirmed = false;
        tracked_objects_.push_back(new_obj);
      }
    }
  }

  // --- Publish target goal ---
  void publishGoal(const geometry_msgs::msg::Point &pt)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose.position = pt;
    goal.pose.orientation.w = 1.0;

    goal_pub_->publish(goal);
    RCLCPP_INFO(this->get_logger(), "🎯 Published target goal: (x=%.2f, y=%.2f)", pt.x, pt.y);
  }

  // --- Members ---
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr object_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<TrackedObject> tracked_objects_;
  std::vector<geometry_msgs::msg::Point> visited_goals_;

  double visible_time_threshold_;
  double moved_distance_threshold_;
  double merge_threshold_;
  double novelty_threshold_;

  double last_robot_x_, last_robot_y_;
  double total_robot_displacement_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectGoalSelector>());
  rclcpp::shutdown();
  return 0;
}
