cmake_minimum_required(VERSION 3.8)
project(swarm_collision_checker)

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
  nav_msgs
  visualization_msgs
)
# find dependencies
foreach(dependency IN ITEMS ${ROS_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()


add_executable(swarm_collision_checker_node 
  src/swarm_collision_checker_node.cpp)
ament_target_dependencies(swarm_collision_checker_node 
  Eigen3
  rclcpp
  nav_msgs
  visualization_msgs
)
target_include_directories(swarm_collision_checker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(swarm_collision_checker_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS swarm_collision_checker_node
  DESTINATION lib/${PROJECT_NAME})

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
