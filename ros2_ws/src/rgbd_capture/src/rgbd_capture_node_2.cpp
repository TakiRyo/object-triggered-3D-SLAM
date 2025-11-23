#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp> // Added for TYPE_32FC1
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
    // --- Parameters ---
    declare_parameter("rgb_topic", "/intel_realsense_r200_rgb/image_raw");
    declare_parameter("depth_topic", "/intel_realsense_r200_depth/depth/image_raw");
    declare_parameter("world_frame", "map");
    declare_parameter("camera_frame", "camera_rgb_optical_frame");
    // Default path (Update this if needed)
    declare_parameter("output_dir", "/home/ros2_env/taki/otslam/3d_model/object_scan");

    get_parameter("rgb_topic", rgb_topic_);
    get_parameter("depth_topic", depth_topic_);
    get_parameter("world_frame", world_frame_);
    get_parameter("camera_frame", camera_frame_);
    get_parameter("output_dir", output_dir_);

    // --- Subscriptions ---
    // RGB Subscription
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
        rgb_topic_, 10, std::bind(&RGBDCaptureNode::rgbCallback, this, _1));
    
    // Depth Subscription
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, 10, std::bind(&RGBDCaptureNode::depthCallback, this, _1));

    // --- Directory Setup ---
    std::filesystem::create_directories(output_dir_ + "/color");
    std::filesystem::create_directories(output_dir_ + "/depth");
    std::filesystem::create_directories(output_dir_ + "/poses");

    RCLCPP_INFO(get_logger(), "--------------------------------------------------");
    RCLCPP_INFO(get_logger(), " RGB-D Capture Node Ready");
    RCLCPP_INFO(get_logger(), " Save Path: %s", output_dir_.c_str());
    RCLCPP_INFO(get_logger(), " Action: Click OpenCV Window -> Press 'SPACE' to save.");
    RCLCPP_INFO(get_logger(), "--------------------------------------------------");

    count_ = 0;
  }

  void spin()
  {
    // Create visualization window
    cv::namedWindow("RGB Preview", cv::WINDOW_AUTOSIZE);

    while (rclcpp::ok())
    {
      rclcpp::spin_some(shared_from_this());

      if (!rgb_.empty()) {
        cv::imshow("RGB Preview", rgb_);
      }

      int key = cv::waitKey(1);
      if (key == 32)  // SPACE Key
      {
        saveFrame();
      }
      else if (key == 27)  // ESC Key
      {
        RCLCPP_INFO(get_logger(), "Exiting...");
        break;
      }
    }
    cv::destroyAllWindows();
  }

private:
  // --- RGB Callback ---
  void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // Convert ROS image to OpenCV BGR image
      rgb_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge RGB exception: %s", e.what());
    }
  }

  // --- Depth Callback (Fixed) ---
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // Force conversion to 32-bit Float (Meters)
      // This handles both 16UC1 (integers) and 32FC1 (floats) inputs correctly
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      depth_ = cv_ptr->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge Depth exception: %s", e.what());
    }
  }

  // --- Save Function ---
  void saveFrame()
  {
    // 1. Check if data exists
    if (rgb_.empty() || depth_.empty()) {
      RCLCPP_WARN(get_logger(), "Waiting for images... (Check topics in Rviz)");
      return;
    }

    // 2. Prepare Filenames
    std::ostringstream color_fn, depth_fn, pose_fn;
    color_fn << output_dir_ << "/color/color_" << std::setw(4) << std::setfill('0') << count_ << ".jpg";
    // NOTE: Using .png for depth to preserve 16-bit accuracy
    depth_fn << output_dir_ << "/depth/depth_" << std::setw(4) << std::setfill('0') << count_ << ".png";
    pose_fn  << output_dir_ << "/poses/pose_"  << std::setw(4) << std::setfill('0') << count_ << ".txt";

    // 3. Save RGB Image
    cv::imwrite(color_fn.str(), rgb_);

    // 4. Process and Save Depth Image (Critical Step)
    // --------------------------------------------------
    cv::Mat depth_f = depth_.clone(); // Work on a copy

    // A. Handle NaNs/Infs (Common in Gazebo) -> Set to 0.0
    cv::patchNaNs(depth_f, 0.0f);

    // B. (Optional) Filter out far background logic
    // Set anything > 5.0 meters to 0.0 (ignore it)
    cv::threshold(depth_f, depth_f, 5.0, 0.0, cv::THRESH_TOZERO_INV);

    // C. Convert Meters (Float) to Millimeters (Uint16)
    // Scale: 1.0m -> 1000mm
    cv::Mat depth_u16;
    depth_f.convertTo(depth_u16, CV_16UC1, 1000.0);

    // D. Save
    cv::imwrite(depth_fn.str(), depth_u16);
    // --------------------------------------------------

    // 5. Get and Save TF Pose
    try {
      geometry_msgs::msg::TransformStamped tf_msg =
          tf_buffer_.lookupTransform(world_frame_, camera_frame_, tf2::TimePointZero);

      // Extract Rotation (Quaternion -> Matrix)
      tf2::Quaternion q(
          tf_msg.transform.rotation.x,
          tf_msg.transform.rotation.y,
          tf_msg.transform.rotation.z,
          tf_msg.transform.rotation.w);
      tf2::Matrix3x3 R(q);

      // Extract Translation
      double x = tf_msg.transform.translation.x;
      double y = tf_msg.transform.translation.y;
      double z = tf_msg.transform.translation.z;

      // Write 4x4 Matrix to file
      std::ofstream f(pose_fn.str());
      if (f.is_open()) {
        f << std::fixed << std::setprecision(6)
          << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << x << "\n"
          << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << y << "\n"
          << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << z << "\n"
          << "0.000000 0.000000 0.000000 1.000000\n";
        f.close();
        RCLCPP_INFO(get_logger(), "Captured Frame: %04d", count_);
        count_++;
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to write pose file!");
      }

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF Error: %s", ex.what());
    }
  }

  // Members
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  
  cv::Mat rgb_;
  cv::Mat depth_; // Keeps raw float data
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  std::string rgb_topic_, depth_topic_;
  std::string world_frame_, camera_frame_;
  std::string output_dir_;
  
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