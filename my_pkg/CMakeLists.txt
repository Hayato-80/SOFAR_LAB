cmake_minimum_required(VERSION 3.5)
project(my_pkg)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
ament_auto_find_build_dependencies()

install(
  DIRECTORY 
  config
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)


# install(PROGRAMS script/my_node_py.py
#   DESTINATION lib/${PROJECT_NAME})
ament_auto_add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(
#     # turtle_tf2_broadcaster
#     geometry_msgs
#     rclcpp
#     tf2
#     tf2_ros
#     turtlesim
# )
# target_link_libraries(my_node
#     tf2_ros
# )


ament_auto_package()
