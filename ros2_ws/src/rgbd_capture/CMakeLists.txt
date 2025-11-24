cmake_minimum_required(VERSION 3.8)
project(rgbd_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. FIND DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)  # <--- FIXED: Added this
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)    # <--- FIXED: Added explicit find
find_package(OpenCV REQUIRED)

# 2. BUILD NODE 1 (Original)
add_executable(rgbd_capture_node src/rgbd_capture_node_2.cpp)
ament_target_dependencies(rgbd_capture_node
  rclcpp
  sensor_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)
# Link OpenCV separately
target_link_libraries(rgbd_capture_node ${OpenCV_LIBRARIES})

# 3. BUILD NODE 2 (Ground Truth Version)
add_executable(rgbd_capture_node_gt src/rgbd_capture_node_gt.cpp)
ament_target_dependencies(rgbd_capture_node_gt
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)
# Link OpenCV separately (FIXED: Do not link to the other node)
target_link_libraries(rgbd_capture_node_gt ${OpenCV_LIBRARIES})

# 4. INSTALL
install(TARGETS 
  rgbd_capture_node 
  rgbd_capture_node_gt  # <--- FIXED: Typo (rgbf -> rgbd)
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()