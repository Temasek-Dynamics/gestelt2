cmake_minimum_required(VERSION 3.5)
project(gestelt_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(gestelt_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)

nav2_package()

set(executable_name controller_server)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/controller_server.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${library_name} PUBLIC
  nav_2d_utils
  px4_msgs
)
target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  gestelt_core::gestelt_core
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${nav_2d_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  tf2_ros::tf2_ros
)
target_link_libraries(${library_name} PRIVATE
  ${lifecycle_msgs_TARGETS}
  rclcpp_components::component
)

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${executable_name} PRIVATE
  rclcpp::rclcpp
  ${library_name}
)

add_library(simple_progress_checker SHARED plugins/simple_progress_checker.cpp)
target_include_directories(simple_progress_checker
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(simple_progress_checker PUBLIC
  nav_2d_utils
)
target_link_libraries(simple_progress_checker PUBLIC
  ${geometry_msgs_TARGETS}
  gestelt_core::gestelt_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
)
target_link_libraries(simple_progress_checker PRIVATE
  nav2_util::nav2_util_core
  pluginlib::pluginlib
)

add_library(pose_progress_checker SHARED plugins/pose_progress_checker.cpp)
target_include_directories(pose_progress_checker
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(pose_progress_checker PUBLIC
  nav_2d_utils
)
target_link_libraries(pose_progress_checker PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  simple_progress_checker
)
target_link_libraries(pose_progress_checker PRIVATE
  angles::angles
  nav2_util::nav2_util_core
  pluginlib::pluginlib
)

add_library(simple_goal_checker SHARED plugins/simple_goal_checker.cpp)
target_include_directories(simple_goal_checker
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(simple_goal_checker PUBLIC
  nav_2d_utils
)
target_link_libraries(simple_goal_checker PUBLIC
  ${geometry_msgs_TARGETS}
  gestelt_core::gestelt_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
)
target_link_libraries(simple_goal_checker PRIVATE
  angles::angles
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  tf2::tf2
)

add_library(stopped_goal_checker SHARED plugins/stopped_goal_checker.cpp)
target_include_directories(stopped_goal_checker
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(stopped_goal_checker PUBLIC
  nav_2d_utils
)
target_link_libraries(stopped_goal_checker PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  simple_goal_checker
)
target_link_libraries(stopped_goal_checker PRIVATE
  nav2_util::nav2_util_core
  pluginlib::pluginlib
)

rclcpp_components_register_nodes(${library_name} "gestelt_controller::ControllerServer")

install(TARGETS simple_progress_checker pose_progress_checker simple_goal_checker stopped_goal_checker ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(simple_progress_checker
  pose_progress_checker
  simple_goal_checker
  stopped_goal_checker
  ${library_name})
ament_export_dependencies(
  geometry_msgs
  gestelt_core
  nav2_msgs
  nav2_util
  nav_2d_msgs
  nav_2d_utils
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  tf2_ros
)
ament_export_targets(${library_name})
pluginlib_export_plugin_description_file(gestelt_core plugins.xml)

ament_package()
