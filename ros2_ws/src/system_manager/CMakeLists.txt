cmake_minimum_required(VERSION 3.8)
project(system_manager)


# 1. FIND DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(otslam_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)

# 2. BUILD EXECUTABLE
add_executable(manager_node src/manager_node.cpp)

# 3. LINK DEPENDENCIES
ament_target_dependencies(manager_node
  rclcpp
  rclcpp_action
  geometry_msgs
  otslam_interfaces
  nav2_msgs
)

add_executable(scanner_node src/scanner_node.cpp)
ament_target_dependencies(scanner_node
  rclcpp
  rclcpp_action
  otslam_interfaces
)

# 4. INSTALL
install(TARGETS 
  manager_node
  scanner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/ 
  DESTINATION include/)

ament_package()