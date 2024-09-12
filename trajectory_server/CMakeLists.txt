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
	sensor_msgs
	gestelt_interfaces
	logger_wrapper)

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
	sensor_msgs
	gestelt_interfaces)

#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/frame_transforms.cpp)
ament_target_dependencies(frame_transforms 
	Eigen3
	geometry_msgs
	sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Trajectory Server
add_executable(trajectory_server_node src/trajectory_server.cpp src/uavsm.cpp)
ament_target_dependencies(trajectory_server_node 
	rclcpp 
	px4_msgs 
	Eigen3
	geometry_msgs
	nav_msgs
	sensor_msgs
	gestelt_interfaces
	logger_wrapper)
target_include_directories(trajectory_server_node PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(trajectory_server_node frame_transforms)
install(TARGETS trajectory_server_node DESTINATION lib/${PROJECT_NAME})

# (For testing) Trajectory Server UAV Command service client 
add_executable(uav_cmd_srv_client src/uav_cmd_srv_client.cpp )
ament_target_dependencies(uav_cmd_srv_client 
	rclcpp 
	gestelt_interfaces)
target_include_directories(uav_cmd_srv_client PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
install(TARGETS uav_cmd_srv_client DESTINATION lib/${PROJECT_NAME})

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
# ament_python_install_package(${PROJECT_NAME})

# # Install Python executables
# install(PROGRAMS
#   src/.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
