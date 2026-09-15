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
find_package(message_filters REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(ora_interfaces REQUIRED)

# Include directories
include_directories(include)

# Create the semantic segmentation layer library
add_library(semantic_segmentation_layer SHARED
  src/semantic_segmentation_layer.cpp
  src/segmentation_buffer.cpp
)

target_include_directories(semantic_segmentation_layer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(semantic_segmentation_layer
  PUBLIC
    message_filters::message_filters
    nav2_costmap_2d::nav2_costmap_2d_core
    nav2_util::nav2_util_core
    nav2_util::nav2_util_core
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_sensor_msgs::tf2_sensor_msgs
    ${vision_msgs_TARGETS}
)

add_executable(gps_waypoint_follower src/gps_waypoint_follower.cpp)
ament_target_dependencies(gps_waypoint_follower
add_executable(navigation_manager
  src/navigation_manager.cpp
  src/gps_waypoint_follower.cpp
)
ament_target_dependencies(navigation_manager
  rclcpp rclcpp_action ament_index_cpp std_msgs std_srvs  geographic_msgs geometry_msgs
  nav2_msgs ora_interfaces
  fusioncore_ros
)

target_link_libraries(navigation_manager
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

# Install semantic segmentation layer library
install(TARGETS semantic_segmentation_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install semantic segmentation header files
install(DIRECTORY include/
  DESTINATION include
)

# Install semantic segmentation layer plugin description file
install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export semantic segmentation layer plugin description file for pluginlib
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
navigation_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
