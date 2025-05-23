cmake_minimum_required(VERSION 3.8)
project(pcd_map_generator)

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
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(nlohmann_json REQUIRED)

set(ROS_DEPENDENCIES
	ament_cmake
	ament_cmake_ros
	rclcpp
  pcl_conversions)

foreach(dependency IN ITEMS ${ROS_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_executable(pcd_map_generator src/pcd_map_generator.cpp)
ament_target_dependencies(pcd_map_generator 
  Eigen3  
  pcl_conversions
  rclcpp)
  target_include_directories(pcd_map_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(pcd_map_generator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(pcd_map_generator
  ${PCL_LIBRARIES}
  # nlohmann_json::nlohmann_json
)

############
# Install ##
############

# Install executables.
install(TARGETS pcd_map_generator
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

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