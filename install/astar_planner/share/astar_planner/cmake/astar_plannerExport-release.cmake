#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "astar_planner::astar_planner" for configuration "Release"
set_property(TARGET astar_planner::astar_planner APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(astar_planner::astar_planner PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libastar_planner.so"
  IMPORTED_SONAME_RELEASE "libastar_planner.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS astar_planner::astar_planner )
list(APPEND _IMPORT_CHECK_FILES_FOR_astar_planner::astar_planner "${_IMPORT_PREFIX}/lib/libastar_planner.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
