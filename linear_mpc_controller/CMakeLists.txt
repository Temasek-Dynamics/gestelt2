cmake_minimum_required(VERSION 3.5)
project(linear_mpc_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(gestelt_core REQUIRED)
find_package(occ_map REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  occ_map
  pluginlib
  nav_msgs
  nav2_util
  gestelt_core
  tf2
  tf2_geometry_msgs
)

set(library_name linear_mpc_controller)

add_library(${library_name} SHARED
        src/regulated_pure_pursuit_controller.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(gestelt_core linear_mpc_controller.xml)

ament_package()

