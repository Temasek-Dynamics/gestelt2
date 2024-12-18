cmake_minimum_required(VERSION 3.8)
project(decomp_util)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# # set default build type
# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE RelWithDebInfo)
# endif()
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

# set(BUILD_SHARED_LIBS ON)

# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIRS} include)

# include(CTest)

# # add_executable(test_seed_decomp test/test_seed_decomp.cpp)
# # target_link_libraries(test_seed_decomp ${Boost_LIBRARIES})
# # add_test(test_seed_decomp test_seed_decomp)

# # add_executable(test_line_segment test/test_line_segment.cpp)
# # target_link_libraries(test_line_segment ${Boost_LIBRARIES})
# # add_test(test_line_segment test_line_segment ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

# # add_executable(test_ellipsoid_decomp test/test_ellipsoid_decomp.cpp)
# # target_link_libraries(test_ellipsoid_decomp ${Boost_LIBRARIES})
# # add_test(test_ellipsoid_decomp test_ellipsoid_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

# # add_executable(test_iterative_decomp test/test_iterative_decomp.cpp)
# # target_link_libraries(test_iterative_decomp ${Boost_LIBRARIES})
# # add_test(test_iterative_decomp test_iterative_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

# install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
#   DESTINATION "share/${PROJECT_NAME}/cmake")

# install(TARGETS
#   RUNTIME DESTINATION bin
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib)

# install(DIRECTORY include/decomp_basis include/decomp_geometry include/decomp_util
#   DESTINATION include)

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

add_library(${PROJECT_NAME} 
  src/${PROJECT_NAME}.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
# ament_target_dependencies(${PROJECT_NAME} 
#   ${ROS_LIBRABRIES}
# )
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

############
# Install ##
############

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION "share/${PROJECT_NAME}/cmake")

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
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# ament_export_dependencies(
#   ${EXPORTED_DEPENDENCIES}
# )

ament_package()
