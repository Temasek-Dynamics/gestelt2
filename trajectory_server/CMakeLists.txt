cmake_minimum_required(VERSION 3.5)
project(trajectory_server)

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

set(ROS_LIBRABRIES
	ament_cmake
	builtin_interfaces
	rclcpp
	rclpy
	px4_msgs
	geometry_msgs
	nav_msgs
	sensor_msgs)

foreach(dependency IN ITEMS ${ROS_LIBRABRIES})
  find_package(${dependency} REQUIRED)
endforeach()

set(EXPORTED_DEPENDENCIES
	ament_cmake
	rclcpp
	rosidl_default_runtime
	eigen3_cmake_module
	Eigen3
	px4_msgs
	geometry_msgs
	sensor_msgs)

#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms 
	Eigen3
	geometry_msgs
	sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# # examples/listeners/sensor_combined_listener
# add_executable(sensor_combined_listener src/examples/listeners/sensor_combined_listener.cpp)
# ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)
# install(TARGETS sensor_combined_listener DESTINATION lib/${PROJECT_NAME})

# # examples/listeners/vehicle_gps_position_listener
# add_executable(vehicle_gps_position_listener src/examples/listeners/vehicle_gps_position_listener.cpp)
# ament_target_dependencies(vehicle_gps_position_listener rclcpp px4_msgs)
# install(TARGETS vehicle_gps_position_listener DESTINATION lib/${PROJECT_NAME})

# # examples/advertisers/debug_vect_advertiser
# add_executable(debug_vect_advertiser src/examples/advertisers/debug_vect_advertiser.cpp)
# ament_target_dependencies(debug_vect_advertiser rclcpp px4_msgs)
# install(TARGETS debug_vect_advertiser DESTINATION lib/${PROJECT_NAME})

# # examples/offboard/offboard_control
# add_executable(offboard_control src/examples/offboard/offboard_control.cpp)
# ament_target_dependencies(offboard_control rclcpp px4_msgs)
# install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

# # examples/offboard/offboard_control_srv
# add_executable(offboard_control_srv src/examples/offboard/offboard_control_srv.cpp)
# ament_target_dependencies(offboard_control_srv rclcpp px4_msgs)
# install(TARGETS offboard_control_srv DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control_srv
add_executable(trajectory_server src/examples/trajectory_server/trajectory_server.cpp)
ament_target_dependencies(trajectory_server 
	rclcpp 
	px4_msgs 
	Eigen3
	geometry_msgs
	nav_msgs
	sensor_msgs)
target_link_libraries(trajectory_server frame_transforms)
install(TARGETS trajectory_server DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(${EXPORTED_DEPENDENCIES})

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(frame_transforms)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS frame_transforms
	EXPORT export_frame_transforms
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# Install tests
# install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


############
# Testing ##
############

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

###########
# Python ##
###########

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# # Install Python executables
# install(PROGRAMS
#   src/.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
