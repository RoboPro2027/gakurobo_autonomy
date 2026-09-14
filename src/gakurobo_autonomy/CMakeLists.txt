cmake_minimum_required(VERSION 3.8)
project(gakurobo_autonomy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(
  mission_runner
  src/mission_runner.cpp
  src/bt_nodes/navigate_to_pose.cpp
  src/bt_nodes/publish_event.cpp
)

target_include_directories(
  mission_runner
  PRIVATE
    include
)

ament_target_dependencies(
  mission_runner
  behaviortree_cpp_v3
  nav2_msgs
  rclcpp
  rclcpp_action
  std_msgs
)

install(
  TARGETS
    mission_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    behavior_trees
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
