cmake_minimum_required(VERSION 3.5)
project(pid_controller_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(pid_controller_node src/pid_controller_node.cpp)
ament_target_dependencies(pid_controller_node rclcpp geometry_msgs)

install(TARGETS
  pid_controller_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
