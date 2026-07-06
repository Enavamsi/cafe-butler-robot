cmake_minimum_required(VERSION 3.8)
project(cafe_butler_bt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cafe_butler_interfaces REQUIRED)

include_directories(include)

add_library(cafe_butler_bt_nodes SHARED
  src/bt_nodes.cpp
)
ament_target_dependencies(cafe_butler_bt_nodes
  rclcpp rclcpp_action behaviortree_cpp nav2_msgs geometry_msgs std_msgs
)

add_executable(butler_bt_navigator
  src/bt_executor_node.cpp
)
target_link_libraries(butler_bt_navigator cafe_butler_bt_nodes)
ament_target_dependencies(butler_bt_navigator
  rclcpp rclcpp_action behaviortree_cpp nav2_msgs geometry_msgs std_msgs cafe_butler_interfaces
)

install(TARGETS
  cafe_butler_bt_nodes
  butler_bt_navigator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behavior_trees config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
