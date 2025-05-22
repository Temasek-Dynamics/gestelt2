#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "logger_wrapper::logger_wrapper" for configuration "Release"
set_property(TARGET logger_wrapper::logger_wrapper APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(logger_wrapper::logger_wrapper PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblogger_wrapper.so"
  IMPORTED_SONAME_RELEASE "liblogger_wrapper.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS logger_wrapper::logger_wrapper )
list(APPEND _IMPORT_CHECK_FILES_FOR_logger_wrapper::logger_wrapper "${_IMPORT_PREFIX}/lib/liblogger_wrapper.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
