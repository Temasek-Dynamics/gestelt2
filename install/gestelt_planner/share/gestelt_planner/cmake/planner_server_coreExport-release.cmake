#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "gestelt_planner::planner_server_core" for configuration "Release"
set_property(TARGET gestelt_planner::planner_server_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(gestelt_planner::planner_server_core PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libplanner_server_core.so"
  IMPORTED_SONAME_RELEASE "libplanner_server_core.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS gestelt_planner::planner_server_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_gestelt_planner::planner_server_core "${_IMPORT_PREFIX}/lib/libplanner_server_core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
