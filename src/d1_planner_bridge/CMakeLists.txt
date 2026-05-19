cmake_minimum_required(VERSION 3.8)
project(d1_planner_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)

include_directories(include)

add_library(d1_trajectory_tracker SHARED
  src/trajectory_tracker.cpp
)
ament_target_dependencies(d1_trajectory_tracker
  rclcpp
  geometry_msgs
  nav_msgs
  quadrotor_msgs
)

add_executable(d1_planner_bridge_node
  src/d1_planner_bridge_node.cpp
)
target_link_libraries(d1_planner_bridge_node d1_trajectory_tracker)
ament_target_dependencies(d1_planner_bridge_node
  rclcpp
  geometry_msgs
  nav_msgs
  quadrotor_msgs
)

install(
  TARGETS d1_trajectory_tracker d1_planner_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
