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

set(ROS_DEPENDENCIES
	ament_cmake
	rclcpp
	rclpy
	geometry_msgs
	nav_msgs
	sensor_msgs
	gestelt_interfaces
	px4_msgs
	tf2_ros
	logger_wrapper)

foreach(dependency IN ITEMS ${ROS_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

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
add_library(
	trajectory_server 
	src/trajectory_server.cpp 
	src/uavsm.cpp
)
ament_target_dependencies(
	trajectory_server
	${ROS_DEPENDENCIES}
)
target_link_libraries(
	trajectory_server
	Eigen3::Eigen
)
target_include_directories(trajectory_server PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(trajectory_server frame_transforms)

add_executable(
	trajectory_server_node 
	src/main.cpp 
)
target_link_libraries(
	trajectory_server_node 
	trajectory_server
	rclcpp::rclcpp
)

# (For testing) Trajectory Server UAV Command service client 
add_executable(uav_cmd_srv_client src/uav_cmd_srv_client.cpp )
ament_target_dependencies(uav_cmd_srv_client 
	rclcpp 
	gestelt_interfaces)
target_include_directories(uav_cmd_srv_client PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

############
# Install ##
############
install(TARGETS 
	trajectory_server_node
DESTINATION lib/${PROJECT_NAME})

# Export information to downstream packages
# ament_export_dependencies()

# ament_export_include_directories(include)

# ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

# ament_export_libraries(frame_transforms)

# Install header files
# install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# install(TARGETS frame_transforms
# 	EXPORT export_frame_transforms
# 	ARCHIVE DESTINATION lib
# 	LIBRARY DESTINATION lib
# 	RUNTIME DESTINATION bin
# 	INCLUDES DESTINATION include
# )

# # Install config files
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)


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


ament_package()
