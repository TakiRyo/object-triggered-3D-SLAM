cmake_minimum_required(VERSION 3.5)
project(gazebo_turtlebot3)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Link and include
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# Executable(s) (edit this if you have custom nodes)
set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
)

# add_executable(custom_drive_node src/turtlebot3_drive.cpp)
# ament_target_dependencies(custom_drive_node ${dependencies})

# # Gazebo plugin examples (only include if you use them)
# add_library(custom_plugin SHARED src/traffic_light_plugin.cpp)
# target_link_libraries(custom_plugin ${GAZEBO_LIBRARIES})

# # Optional: more plugins
# # add_library(obstacle1 SHARED src/obstacle1.cpp)
# # target_link_libraries(obstacle1 ${GAZEBO_LIBRARIES})

# # Install everything you need
# install(TARGETS
#   custom_drive_node
#   custom_plugin
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY
  launch
  models
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

# Export
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_package()
