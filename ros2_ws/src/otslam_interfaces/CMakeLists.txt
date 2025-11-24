cmake_minimum_required(VERSION 3.8)
project(otslam_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ScanObject.action"
)

ament_package()
