cmake_minimum_required(VERSION 3.8)
project(rtab_3d_scan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtabmap_ros REQUIRED)

# CRITICAL: Install the launch directory so ros2 launch can find it
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
