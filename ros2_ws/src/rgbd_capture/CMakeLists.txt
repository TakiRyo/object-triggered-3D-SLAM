cmake_minimum_required(VERSION 3.8)
project(rgbd_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)


# Build the node
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

target_link_libraries(rgbd_capture_node ${OpenCV_LIBRARIES})

# Install
install(TARGETS rgbd_capture_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
