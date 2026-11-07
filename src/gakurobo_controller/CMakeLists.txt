cmake_minimum_required(VERSION 3.8)
project(gakurobo_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(gakurobo_arrival_controller SHARED src/arrival_controller.cpp)
target_include_directories(gakurobo_arrival_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(gakurobo_arrival_controller PUBLIC cxx_std_17)
ament_target_dependencies(gakurobo_arrival_controller
  geometry_msgs nav2_core nav2_costmap_2d nav2_util nav_msgs pluginlib rclcpp
  rclcpp_lifecycle std_msgs tf2_ros)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS gakurobo_arrival_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_arrival_math test/test_arrival_math.cpp)
  target_include_directories(test_arrival_math PRIVATE include)
endif()

ament_package()
