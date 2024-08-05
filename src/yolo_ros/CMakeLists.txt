cmake_minimum_required(VERSION 3.5)
project(yolo_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(yolo_node src/yolo_node.cpp)
ament_target_dependencies(yolo_node rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
  yolo_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
