cmake_minimum_required(VERSION 3.8)
project(fake_sensor)

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
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fake_sensor_node 
  src/fake_sensor_node.cpp 
  src/fake_sensor.cpp 
  include/fake_sensor/sensor_renderer.hpp
  include/fake_sensor/fake_sensor.hpp
)
ament_target_dependencies(fake_sensor_node 
  rclcpp
  tf2
  tf2_ros
  pcl_conversions
  sensor_msgs
  nav_msgs
)
target_include_directories(fake_sensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(fake_sensor_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(fake_sensor_node
  ${PCL_LIBRARIES}
)

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

install(TARGETS fake_sensor_node
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
