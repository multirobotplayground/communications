cmake_minimum_required(VERSION 3.8)
project(map_stitching)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(comm_msgs REQUIRED)

include_directories(include)

# add_library(${PROJECT_NAME}_lib)
# target_link_libraries(${PROJECT_NAME}_lib)
# ament_target_dependencies(${PROJECT_NAME}_lib
#   rclcpp
#   std_msgs
#   geometry_msgs
#   nav_msgs
#   std_srvs
#   tf2
#   tf2_geometry_msgs
#   nav2_msgs
#   comm_msgs
# )

add_executable(map_stitching_node src/MapStitching.cpp)
# target_link_libraries(map_stitching_node ${PROJECT_NAME}_lib)
ament_target_dependencies(map_stitching_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  nav2_msgs
  comm_msgs
)

add_executable(map_service_node src/MapService.cpp)
ament_target_dependencies(map_service_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2_geometry_msgs
  nav2_msgs
  comm_msgs
)

install(TARGETS map_stitching_node map_service_node DESTINATION lib/${PROJECT_NAME})

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
