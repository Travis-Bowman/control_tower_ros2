cmake_minimum_required(VERSION 3.8)
project(control_tower_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelCommands.msg"
)

ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY control_tower_ros2
  DESTINATION lib/python3.12/site-packages
)

install(
  PROGRAMS
    control_tower_ros2/control_tower_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME control_tower_node
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()