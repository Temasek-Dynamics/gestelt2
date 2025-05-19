#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "occ_map::occ_map" for configuration "Release"
set_property(TARGET occ_map::occ_map APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(occ_map::occ_map PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libocc_map.so"
  IMPORTED_SONAME_RELEASE "libocc_map.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS occ_map::occ_map )
list(APPEND _IMPORT_CHECK_FILES_FOR_occ_map::occ_map "${_IMPORT_PREFIX}/lib/libocc_map.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
