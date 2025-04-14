cmake_minimum_required(VERSION 3.8)
project(gestelt_core)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(ROS_DEPENDENCIES
	ament_cmake
	ament_cmake_ros
  pluginlib
  rclcpp
  rclcpp_lifecycle
  nav2_util
  nav_msgs
  geometry_msgs
  tf2_ros
  occ_map
  px4_msgs
)

foreach(dependency IN ITEMS ${ROS_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(gestelt_core INTERFACE)
add_library(gestelt_core::gestelt_core ALIAS gestelt_core)
target_include_directories(gestelt_core
  INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(
  gestelt_core INTERFACE
  ${ROS_DEPENDENCIES}
)

# target_link_libraries(gestelt_core INTERFACE
#   pluginlib::pluginlib
#   rclcpp::rclcpp
#   rclcpp_lifecycle::rclcpp_lifecycle
#   nav2_util::nav2_util_core
#   tf2_ros::tf2_ros
#   ${geometry_msgs_TARGETS}
#   ${nav_msgs_TARGETS}
#   occ_map::occ_map
# )

install(TARGETS gestelt_core
  EXPORT gestelt_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
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

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  ${ROS_DEPENDENCIES}
)
ament_export_targets(
  gestelt_core
)

ament_package()
