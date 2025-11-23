#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <filesystem>
#include <fstream>
#include <iomanip>

using std::placeholders::_1;

class RGBDCaptureNode : public rclcpp::Node
{
public:
  RGBDCaptureNode()
  : Node("rgbd_capture_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // parameters
    declare_parameter("rgb_topic", "/intel_realsense_r200_rgb/image_raw");
    declare_parameter("depth_topic", "/intel_realsense_r200_depth/depth/image_raw");
    declare_parameter("world_frame", "map");
    declare_parameter("camera_frame", "camera_rgb_optical_frame");
    declare_parameter("output_dir", "/home/ros2_env/taki/otslam/3d_model/object_scan");

    get_parameter("rgb_topic", rgb_topic_);
    get_parameter("depth_topic", depth_topic_);
    get_parameter("world_frame", world_frame_);
    get_parameter("camera_frame", camera_frame_);
    get_parameter("output_dir", output_dir_);

    // subscriptions
    rgb_sub_   = create_subscription<sensor_msgs::msg::Image>(
        rgb_topic_, 10, std::bind(&RGBDCaptureNode::rgbCallback, this, _1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, 10, std::bind(&RGBDCaptureNode::depthCallback, this, _1));

    std::filesystem::create_directories(output_dir_ + "/color");
    std::filesystem::create_directories(output_dir_ + "/depth");
    std::filesystem::create_directories(output_dir_ + "/poses");

    RCLCPP_INFO(get_logger(),
      "Ready. Press SPACE in the OpenCV window to capture (ESC to exit).");

    count_ = 0;
  }

  void spin()
  {
    cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);

    while (rclcpp::ok())
    {
      rclcpp::spin_some(shared_from_this());
      if (!rgb_.empty())
        cv::imshow("RGB", rgb_);

      int key = cv::waitKey(1);
      if (key == 32)  // SPACE
      {
        saveFrame();
      }
      else if (key == 27)  // ESC
      {
        break;
      }
    }
    cv::destroyAllWindows();
  }

private:
  void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      rgb_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      depth_ = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void saveFrame()
  {
    if (rgb_.empty() || depth_.empty()) {
      RCLCPP_WARN(get_logger(), "Images not ready yet.");
      return;
    }

    // filenames
    std::ostringstream color_path, depth_path, pose_path;
    color_path << output_dir_ << "/color/color_" << std::setw(4)
              << std::setfill('0') << count_ << ".png";
    depth_path << output_dir_ << "/depth/depth_" << std::setw(4)
              << std::setfill('0') << count_ << ".png";
    pose_path  << output_dir_ << "/poses/pose_"  << std::setw(4)
              << std::setfill('0') << count_ << ".txt";

    cv::imwrite(color_path.str(), rgb_);

    // -------------------------------
    // FIXED DEPTH SAVING (CORRECT)
    // -------------------------------
    cv::Mat depth_mm_u16;
    cv::Mat valid = (depth_ > 0) & (depth_ < 5.0);  // valid range 0–5 m
    cv::Mat depth_clean = cv::Mat::zeros(depth_.size(), depth_.type());
    depth_.copyTo(depth_clean, valid);
    depth_clean.convertTo(depth_mm_u16, CV_16UC1, 1000.0);  // meters→mm
    cv::imwrite(depth_path.str(), depth_mm_u16);

    RCLCPP_INFO(get_logger(), "Saved images %d", count_);


    // get TF pose
    try {
      geometry_msgs::msg::TransformStamped tf_msg =
          tf_buffer_.lookupTransform(world_frame_, camera_frame_, tf2::TimePointZero);

      tf2::Quaternion q(
          tf_msg.transform.rotation.x,
          tf_msg.transform.rotation.y,
          tf_msg.transform.rotation.z,
          tf_msg.transform.rotation.w);

      tf2::Matrix3x3 R(q);
      double r00 = R[0][0];
      double r01 = R[0][1];
      double r02 = R[0][2];
      double r10 = R[1][0];
      double r11 = R[1][1];
      double r12 = R[1][2];
      double r20 = R[2][0];
      double r21 = R[2][1];
      double r22 = R[2][2];


      double x = tf_msg.transform.translation.x;
      double y = tf_msg.transform.translation.y;
      double z = tf_msg.transform.translation.z;

      std::ofstream f(pose_path.str());
      f << std::fixed << std::setprecision(6)
        << r00 << " " << r01 << " " << r02 << " " << x << "\n"
        << r10 << " " << r11 << " " << r12 << " " << y << "\n"
        << r20 << " " << r21 << " " << r22 << " " << z << "\n"
        << "0.000000 0.000000 0.000000 1.000000\n";
      f.close();

      RCLCPP_INFO(get_logger(), "Saved pose_%04d.txt", count_);
      count_++;

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
    }
  }

  // members
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  cv::Mat rgb_, depth_;
  cv_bridge::CvImagePtr cv_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string rgb_topic_, depth_topic_, world_frame_, camera_frame_, output_dir_;
  int count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RGBDCaptureNode>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}
