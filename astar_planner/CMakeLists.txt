cmake_minimum_required(VERSION 3.8)
project(astar_planner)

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
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav2_util
  tf2_ros
  nav_msgs
  gestelt_core
  occ_map
)

foreach(dependency IN ITEMS ${ROS_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

set(library_name astar_planner)

add_library(${library_name} SHARED
  src/astar_planner.cpp
  src/astar.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(
  ${library_name}
  ${ROS_DEPENDENCIES}
)
target_link_libraries(
  ${library_name} 
  Eigen3::Eigen
)

pluginlib_export_plugin_description_file(gestelt_core global_planner_plugin.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${library_name} PRIVATE "ASTAR_PLANNER_BUILDING_LIBRARY")

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(FILES global_planner_plugin.xml
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

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  ${ROS_DEPENDENCIES}
)
ament_export_targets(${library_name})
ament_package()
