cmake_minimum_required(VERSION 3.8)
project(gestelt_planner)

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
set(EXTERNAL_LIBRARIES
	eigen3_cmake_module
	Eigen3
)

set(ROS_LIBRARIES
	ament_cmake
  rclcpp
  rclcpp_components

  nav2_msgs
  geometry_msgs
  lifecycle_msgs
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros

  gestelt_core
  occ_map
)

# find dependencies
foreach(dependency IN ITEMS ${EXTERNAL_LIBRARIES})
  find_package(${dependency} REQUIRED)
endforeach()

foreach(dependency IN ITEMS ${ROS_LIBRARIES})
  find_package(${dependency} REQUIRED)
endforeach()

set(executable_name planner_server)
set(library_name ${executable_name}_core)

add_library(${library_name}
  src/planner_server.cpp
)
target_compile_features(${library_name} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(${library_name}
  ${ROS_LIBRARIES}
)
target_link_libraries(${library_name}
  Eigen3::Eigen
)

add_executable(${executable_name}
  src/main.cpp
)
target_link_libraries(
  ${executable_name} 
  ${library_name} 
  rclcpp::rclcpp
)

rclcpp_components_register_nodes(${library_name} "gestelt_planner::PlannerServer")

install(TARGETS ${library_name}
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
  geometry_msgs
  lifecycle_msgs
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  occ_map
)
ament_export_targets(${library_name})
ament_package()

