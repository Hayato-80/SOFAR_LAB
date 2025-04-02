cmake_minimum_required(VERSION 3.5)
project(my_pkg)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
ament_auto_find_build_dependencies()

install(
  DIRECTORY 
  config
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS script/my_node_py.py
  DESTINATION lib/${PROJECT_NAME})
ament_auto_add_executable(my_node src/my_node.cpp)

ament_auto_package()
