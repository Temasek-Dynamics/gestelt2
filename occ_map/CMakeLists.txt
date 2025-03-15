cmake_minimum_required(VERSION 3.8)
project(occ_map)

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
  PCL
	eigen3_cmake_module
	Eigen3)

set(ROS_LIBRABRIES
	ament_cmake
	ament_cmake_ros
	rclcpp
  pcl_conversions
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  logger_wrapper
  bonxai
  tf2_eigen
  nav2_util
  ikd_tree
)

foreach(dependency IN ITEMS ${ROS_LIBRABRIES})
  find_package(${dependency} REQUIRED)
endforeach()

foreach(dependency IN ITEMS ${EXTERNAL_LIBRARIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(occ_map SHARED
  src/occ_map.cpp 
)
add_library(occ_map::occ_map ALIAS occ_map)
ament_target_dependencies(${PROJECT_NAME} 
  ${ROS_LIBRABRIES}
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "${PROJECT_NAME}_BUILDING_LIBRARY")

############
# Install ##
############

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  occ_map
)
ament_export_targets(
  occ_map
)

ament_export_dependencies(
  PCL
  pcl_conversions
  bonxai
  tf2_eigen
  ikd_tree
  logger_wrapper
)

ament_package()
