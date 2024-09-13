#ifndef LOGGER_WRAPPER__VISIBILITY_CONTROL_H_
#define LOGGER_WRAPPER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LOGGER_WRAPPER_EXPORT __attribute__ ((dllexport))
    #define LOGGER_WRAPPER_IMPORT __attribute__ ((dllimport))
  #else
    #define LOGGER_WRAPPER_EXPORT __declspec(dllexport)
    #define LOGGER_WRAPPER_IMPORT __declspec(dllimport)
  #endif
  #ifdef LOGGER_WRAPPER_BUILDING_LIBRARY
    #define LOGGER_WRAPPER_PUBLIC LOGGER_WRAPPER_EXPORT
  #else
    #define LOGGER_WRAPPER_PUBLIC LOGGER_WRAPPER_IMPORT
  #endif
  #define LOGGER_WRAPPER_PUBLIC_TYPE LOGGER_WRAPPER_PUBLIC
  #define LOGGER_WRAPPER_LOCAL
#else
  #define LOGGER_WRAPPER_EXPORT __attribute__ ((visibility("default")))
  #define LOGGER_WRAPPER_IMPORT
  #if __GNUC__ >= 4
    #define LOGGER_WRAPPER_PUBLIC __attribute__ ((visibility("default")))
    #define LOGGER_WRAPPER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LOGGER_WRAPPER_PUBLIC
    #define LOGGER_WRAPPER_LOCAL
  #endif
  #define LOGGER_WRAPPER_PUBLIC_TYPE
#endif

#endif  // LOGGER_WRAPPER__VISIBILITY_CONTROL_H_
