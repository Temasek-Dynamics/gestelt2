cmake_minimum_required(VERSION 3.8)
project(voxel_map)

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
find_package(PCL REQUIRED COMPONENTS common io)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(ROS_LIBRABRIES
	ament_cmake
	ament_cmake_ros
	rclcpp
  pcl_conversions
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  message_filters
  logger_wrapper
  bonxai)

set(EXPORTED_DEPENDENCIES
  bonxai
  PCL
  pcl_conversions
)

foreach(dependency IN ITEMS ${ROS_LIBRABRIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(voxel_map src/voxel_map.cpp)
add_library(voxel_map::voxel_map ALIAS voxel_map)
ament_target_dependencies(voxel_map 
  PCL
  Eigen3  
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  tf2_ros
  message_filters
  logger_wrapper
  bonxai)
target_compile_features(voxel_map PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(voxel_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(voxel_map PRIVATE "VOXEL_MAP_BUILDING_LIBRARY")

############
# Install ##
############

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS voxel_map
  EXPORT export_${PROJECT_NAME}
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

ament_export_dependencies(
  ${EXPORTED_DEPENDENCIES}
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  voxel_map
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
