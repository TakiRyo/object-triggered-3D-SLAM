cmake_minimum_required(VERSION 3.8)
project(system_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. FIND DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(otslam_interfaces REQUIRED) # Your Action Interface

# 2. BUILD EXECUTABLE
add_executable(manager_node src/manager_node.cpp)

# 3. LINK DEPENDENCIES
ament_target_dependencies(manager_node
  rclcpp
  rclcpp_action
  geometry_msgs
  otslam_interfaces
)

# 4. INSTALL
install(TARGETS 
  manager_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()