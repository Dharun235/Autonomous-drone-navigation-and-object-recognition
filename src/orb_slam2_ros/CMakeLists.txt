cmake_minimum_required(VERSION 3.5)
project(orb_slam2_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(orb_slam2_node src/orb_slam2_node.cpp)
ament_target_dependencies(orb_slam2_node rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
  orb_slam2_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
