#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "bonxai::bonxai" for configuration "Release"
set_property(TARGET bonxai::bonxai APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(bonxai::bonxai PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libbonxai.so"
  IMPORTED_SONAME_RELEASE "libbonxai.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS bonxai::bonxai )
list(APPEND _IMPORT_CHECK_FILES_FOR_bonxai::bonxai "${_IMPORT_PREFIX}/lib/libbonxai.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
