cmake_minimum_required(VERSION 3.5)
project(linear_mpc_controller)

set (EXTERNAL_DEPENDENCIES
  Eigen3
  OsqpEigen
)

set(ROS_DEPENDENCIES
  ament_cmake
  nav2_common
  rclcpp
  geometry_msgs
  occ_map
  pluginlib
  nav_msgs
  nav2_util
  gestelt_core
  tf2
  tf2_geometry_msgs
  decomp_util
  decomp_ros_msgs
)

foreach(dependency IN ITEMS ${EXTERNAL_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

foreach(dependency IN ITEMS ${ROS_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(library_name linear_mpc_controller)
add_library(${library_name} SHARED
        src/linear_mpc_controller.cpp)
ament_target_dependencies(${library_name}
  ${ROS_DEPENDENCIES}
)
target_link_libraries(${library_name}
  Eigen3::Eigen
  osqp::osqp
  OsqpEigen::OsqpEigen
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${ROS_DEPENDENCIES})

pluginlib_export_plugin_description_file(gestelt_core linear_mpc_controller.xml)

ament_package()

