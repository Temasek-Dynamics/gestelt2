cmake_minimum_required(VERSION 3.8)
project(dynamic_voronoi)

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
find_package(ament_cmake_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(dynamic_voronoi
  include/dynamic_voronoi/dynamic_voronoi.hpp
  include/dynamic_voronoi/bucketedqueue.hpp
  include/dynamic_voronoi/bucketedqueue.h
  src/dynamic_voronoi.cpp
)
add_library(dynamic_voronoi::dynamic_voronoi ALIAS dynamic_voronoi)
ament_target_dependencies( dynamic_voronoi
  Eigen3
)
target_compile_features(dynamic_voronoi PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(dynamic_voronoi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(dynamic_voronoi PRIVATE "DYNAMIC_VORONOI_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS dynamic_voronoi
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

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  dynamic_voronoi
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
