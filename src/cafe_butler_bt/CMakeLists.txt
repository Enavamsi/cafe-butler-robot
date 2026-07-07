cmake_minimum_required(VERSION 3.8)
project(cafe_butler_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)

install(TARGETS
  butler_bt_navigator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behavior_trees config launch 
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
