cmake_minimum_required(VERSION 3.8)
project(mock_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(comm_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

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

add_library(${PROJECT_NAME}_lib src/MockComm.cpp)
target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  rclcpp_action
  nav2_msgs
  std_srvs
  comm_msgs
  tf2)

add_executable(mock_com_node src/MockComm.cpp)
target_link_libraries(mock_com_node ${PROJECT_NAME}_lib)
ament_target_dependencies(mock_com_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  rclcpp_action
  nav2_msgs
  std_srvs
  comm_msgs
  tf2)

install(TARGETS mock_com_node DESTINATION lib/${PROJECT_NAME})

ament_package()
