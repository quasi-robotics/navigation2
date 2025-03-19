cmake_minimum_required(VERSION 3.5)
project(nav2_waypoint_follower)

# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_localization REQUIRED)
find_package(geographic_msgs REQUIRED)

nav2_package()

include_directories(
  include
)

set(executable_name waypoint_follower)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/waypoint_follower.cpp
)
target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  ${geographic_msgs_TARGETS}
  nav2_core::nav2_core
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
)
target_link_libraries(${library_name} PRIVATE
  rclcpp_components::component
)

# Extract the major version (first number before the dot)
string(REGEX MATCH "^[0-9]+" ROS2_MAJOR_VERSION "${rclcpp_VERSION}")
# Convert to integer
if(ROS2_MAJOR_VERSION GREATER_EQUAL 29)
    message(STATUS "ROS2_MAJOR_VERSION (${ROS2_MAJOR_VERSION}) is greater than 29 (Kilted and newer).")
    target_link_libraries(${library_name} PUBLIC robot_localization::rl_lib)
else()
    message(STATUS "ROS2_MAJOR_VERSION (${ROS2_MAJOR_VERSION}) is NOT greater than 29 (Jazzy and older).")
    ament_target_dependencies(${library_name} PUBLIC robot_localization)
endif()

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(wait_at_waypoint SHARED plugins/wait_at_waypoint.cpp)
ament_target_dependencies(wait_at_waypoint ${dependencies})

add_library(photo_at_waypoint SHARED plugins/photo_at_waypoint.cpp)
ament_target_dependencies(photo_at_waypoint ${dependencies})

add_library(input_at_waypoint SHARED plugins/input_at_waypoint.cpp)
ament_target_dependencies(input_at_waypoint ${dependencies})

rclcpp_components_register_nodes(${library_name} "nav2_waypoint_follower::WaypointFollower")

install(TARGETS ${library_name} wait_at_waypoint photo_at_waypoint input_at_waypoint
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(wait_at_waypoint photo_at_waypoint input_at_waypoint ${library_name})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_waypoint_follower plugins.xml)

ament_package()
