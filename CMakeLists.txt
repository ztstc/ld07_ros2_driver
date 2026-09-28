cmake_minimum_required(VERSION 3.10)
project(ld07_ros2_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(ld07_node
  src/ld07_node.cpp
  src/ld07_driver.cpp
)
ament_target_dependencies(ld07_node rclcpp sensor_msgs std_srvs)

install(TARGETS
  ld07_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()