cmake_minimum_required(VERSION 3.8)
project(rgbd_scan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. FIND DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(otslam_interfaces REQUIRED) # Your Action Interface
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 2. BUILD EXECUTABLE
add_executable(scan_node src/scan_node.cpp)

# 3. LINK ROS DEPENDENCIES
ament_target_dependencies(scan_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  cv_bridge
  image_transport
  otslam_interfaces
  tf2
  tf2_geometry_msgs
)

# 4. LINK EXTERNAL DEPENDENCIES (OpenCV)
target_link_libraries(scan_node ${OpenCV_LIBRARIES})

# 5. INSTALL
install(TARGETS 
  scan_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()