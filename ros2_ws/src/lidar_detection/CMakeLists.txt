cmake_minimum_required(VERSION 3.8)
project(lidar_detection)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)


# publish object and wall cluster
add_executable(lidar_cluster_publisher src/lidar_cluster_publisher.cpp)
# estimate object size and publishe marker. 
add_executable(object_goal_selector src/4_object_goal_selector.cpp)
# publish goal node.
add_executable(send_goal_node src/4_send_goal_node.cpp)


ament_target_dependencies(lidar_cluster_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(send_goal_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  visualization_msgs
)

ament_target_dependencies(object_goal_selector
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)


install(
  DIRECTORY worlds launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  lidar_cluster_publisher
  send_goal_node
  object_goal_selector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
