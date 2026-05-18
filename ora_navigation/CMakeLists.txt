cmake_minimum_required(VERSION 3.8)
project(ora_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(fusioncore_ros REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(gps_waypoint_follower src/gps_waypoint_follower.cpp)
ament_target_dependencies(gps_waypoint_follower
  rclcpp rclcpp_action ament_index_cpp std_msgs std_srvs  geographic_msgs geometry_msgs
  nav2_msgs
  fusioncore_ros
)

target_link_libraries(gps_waypoint_follower
  yaml-cpp
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

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  gps_waypoint_follower
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
