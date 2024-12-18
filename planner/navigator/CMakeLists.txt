cmake_minimum_required(VERSION 3.8)
project(navigator)

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
  # nlohmann_json
)

set(ROS_LIBRABRIES
	ament_cmake
	ament_cmake_ros
  rclcpp
  # transformations
  tf2
  tf2_ros
  # Messages
  nav_msgs
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  px4_msgs
  gestelt_interfaces
  minco_interfaces
  # Mapping and planning
  voxel_map
  dynamic_voronoi
  space_time_astar
  planner_utils
  polytope_sfc_gen
  # Controller
  pvaj_mpc
  minco_traj_gen
  # utils
  logger_wrapper
  viz_helper
)

# find dependencies
foreach(dependency IN ITEMS ${ROS_LIBRABRIES})
  find_package(${dependency} REQUIRED)
endforeach()

foreach(dependency IN ITEMS ${EXTERNAL_LIBRARIES})
  find_package(${dependency} REQUIRED)
endforeach()

# Add executable

add_executable(navigator_node 
  src/navigator_node.cpp 
  src/navigator.cpp)
ament_target_dependencies(navigator_node 
  ${ROS_LIBRABRIES}
)
target_include_directories(navigator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(navigator_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(navigator_node
  Eigen3::Eigen
  # nlohmann_json::nlohmann_json
)

install(TARGETS navigator_node
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
