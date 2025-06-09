#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "decomp_util::decomp_util" for configuration "Release"
set_property(TARGET decomp_util::decomp_util APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(decomp_util::decomp_util PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libdecomp_util.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS decomp_util::decomp_util )
list(APPEND _IMPORT_CHECK_FILES_FOR_decomp_util::decomp_util "${_IMPORT_PREFIX}/lib/libdecomp_util.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
