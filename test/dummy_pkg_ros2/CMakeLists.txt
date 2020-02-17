cmake_minimum_required(VERSION 3.5)
project(dummy_pkg_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(dummy_binary main.cpp)
ament_target_dependencies(dummy_binary "rclcpp")
install(TARGETS dummy_binary DESTINATION bin)
ament_package()
