#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ikd_tree::ikd_tree" for configuration "Release"
set_property(TARGET ikd_tree::ikd_tree APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ikd_tree::ikd_tree PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libikd_tree.so"
  IMPORTED_SONAME_RELEASE "libikd_tree.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ikd_tree::ikd_tree )
list(APPEND _IMPORT_CHECK_FILES_FOR_ikd_tree::ikd_tree "${_IMPORT_PREFIX}/lib/libikd_tree.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
