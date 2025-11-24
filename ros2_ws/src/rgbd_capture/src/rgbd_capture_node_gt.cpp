#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/odometry.hpp> // ADDED: To read Ground Truth
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>

// Math headers for quaternion -> matrix conversion
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

class RGBDCaptureNode : public rclcpp::Node
{
public:
  RGBDCaptureNode()
  : Node("rgbd_capture_node")
  {
    // --- Parameters ---
    declare_parameter("rgb_topic", "/intel_realsense_r200_rgb/image_raw");
    declare_parameter("depth_topic", "/intel_realsense_r200_depth/depth/image_raw");
    // New Parameter for Ground Truth
    declare_parameter("pose_topic", "/ground_truth_odom"); 
    declare_parameter("output_dir", "/home/ros2_env/taki/otslam/3d_model/object_scan");

    get_parameter("rgb_topic", rgb_topic_);
    get_parameter("depth_topic", depth_topic_);
    get_parameter("pose_topic", pose_topic_);
    get_parameter("output_dir", output_dir_);

    // --- Subscriptions ---
    rgb_sub_   = create_subscription<sensor_msgs::msg::Image>(
        rgb_topic_, 10, std::bind(&RGBDCaptureNode::rgbCallback, this, _1));
    
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, 10, std::bind(&RGBDCaptureNode::depthCallback, this, _1));

    // NEW: Subscribe to Ground Truth (God Mode)
    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        pose_topic_, 10, std::bind(&RGBDCaptureNode::poseCallback, this, _1));

    // --- Directory Setup ---
    std::filesystem::create_directories(output_dir_ + "/color");
    std::filesystem::create_directories(output_dir_ + "/depth");
    std::filesystem::create_directories(output_dir_ + "/poses");

    RCLCPP_INFO(get_logger(), "--- RGB-D Capture Node (Ground Truth Mode) ---");
    RCLCPP_INFO(get_logger(), "Listening to Pose: %s", pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Press SPACE in OpenCV window to capture.");

    count_ = 0;
    has_pose_ = false;
  }

  void spin()
  {
    cv::namedWindow("RGB Preview", cv::WINDOW_AUTOSIZE);

    while (rclcpp::ok())
    {
      rclcpp::spin_some(shared_from_this());

      if (!rgb_.empty()) cv::imshow("RGB Preview", rgb_);

      int key = cv::waitKey(1);
      if (key == 32) // SPACE
      {
        saveFrame();
      }
      else if (key == 27) // ESC
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
      RCLCPP_ERROR(get_logger(), "cv_bridge RGB: %s", e.what());
    }
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // Float32 (Meters)
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      depth_ = cv_ptr->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge Depth: %s", e.what());
    }
  }

  // --- NEW: Pose Callback ---
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_msg_ = *msg;
    has_pose_ = true;
  }

  void saveFrame()
  {
    if (rgb_.empty() || depth_.empty()) {
      RCLCPP_WARN(get_logger(), "No images yet.");
      return;
    }

    if (!has_pose_) {
      RCLCPP_WARN(get_logger(), "No Ground Truth Pose yet! Move robot slightly?");
      return;
    }

    // --- 1. Save Images (Same as before) ---
    std::ostringstream color_fn, depth_fn, pose_fn;
    color_fn << output_dir_ << "/color/gt_color_" << std::setw(4) << std::setfill('0') << count_ << ".png";
    depth_fn << output_dir_ << "/depth/gt_depth_" << std::setw(4) << std::setfill('0') << count_ << ".png";
    pose_fn  << output_dir_ << "/poses/gt_pose_"  << std::setw(4) << std::setfill('0') << count_ << ".txt";

    cv::imwrite(color_fn.str(), rgb_);

    // Depth Processing
    cv::Mat depth_f = depth_.clone();
    cv::patchNaNs(depth_f, 0.0f);
    cv::threshold(depth_f, depth_f, 5.0, 0.0, cv::THRESH_TOZERO_INV);
    cv::Mat depth_u16;
    depth_f.convertTo(depth_u16, CV_16UC1, 1000.0);
    cv::imwrite(depth_fn.str(), depth_u16);

    // --- 2. Save Pose from Topic (NOT TF) ---
    
    // Get latest pose safely
    nav_msgs::msg::Odometry pose_data;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose_data = current_pose_msg_;
    }

    // Convert Quaternion to Matrix
    tf2::Quaternion q(
        pose_data.pose.pose.orientation.x,
        pose_data.pose.pose.orientation.y,
        pose_data.pose.pose.orientation.z,
        pose_data.pose.pose.orientation.w);
    
    tf2::Matrix3x3 R(q);

    double x = pose_data.pose.pose.position.x;
    double y = pose_data.pose.pose.position.y;
    double z = pose_data.pose.pose.position.z;

    // Write to file
    std::ofstream f(pose_fn.str());
    if (f.is_open()) {
      f << std::fixed << std::setprecision(6)
        << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << x << "\n"
        << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << y << "\n"
        << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << z << "\n"
        << "0.000000 0.000000 0.000000 1.000000\n";
      f.close();
      RCLCPP_INFO(get_logger(), "Saved Frame %04d (using Ground Truth)", count_);
      count_++;
    }
  }

  // Members
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_; // NEW

  cv::Mat rgb_, depth_;
  nav_msgs::msg::Odometry current_pose_msg_; // Stores the latest perfect pose
  bool has_pose_;
  std::mutex pose_mutex_;
  
  std::string rgb_topic_, depth_topic_, pose_topic_, output_dir_;
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