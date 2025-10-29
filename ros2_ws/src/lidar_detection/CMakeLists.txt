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


add_executable(send_goal_node src/2_send_goal_node.cpp)

add_executable(object_goal_selector src/3_object_goal_selector.cpp)

# 
add_executable(visiting_point_selector src/2_visiting_point_selector.cpp)

# publish object and wall cluster
add_executable(lidar_cluster_publisher src/lidar_cluster_publisher.cpp)

# estimate wall and room shape. but it doesn't work well. 
add_executable(room_shape_estimator src/2_room_shape_estimator.cpp)



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
)

ament_target_dependencies(object_goal_selector
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

ament_target_dependencies(visiting_point_selector
  rclcpp
  geometry_msgs
  visualization_msgs 
  tf2_ros 
  tf2_geometry_msgs
  )

  ament_target_dependencies(room_shape_estimator
  rclcpp
  sensor_msgs
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
  visiting_point_selector
  room_shape_estimator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
