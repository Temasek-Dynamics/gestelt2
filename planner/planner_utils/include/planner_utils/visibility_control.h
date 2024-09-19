#ifndef PLANNER_UTILS__VISIBILITY_CONTROL_H_
#define PLANNER_UTILS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PLANNER_UTILS_EXPORT __attribute__ ((dllexport))
    #define PLANNER_UTILS_IMPORT __attribute__ ((dllimport))
  #else
    #define PLANNER_UTILS_EXPORT __declspec(dllexport)
    #define PLANNER_UTILS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PLANNER_UTILS_BUILDING_LIBRARY
    #define PLANNER_UTILS_PUBLIC PLANNER_UTILS_EXPORT
  #else
    #define PLANNER_UTILS_PUBLIC PLANNER_UTILS_IMPORT
  #endif
  #define PLANNER_UTILS_PUBLIC_TYPE PLANNER_UTILS_PUBLIC
  #define PLANNER_UTILS_LOCAL
#else
  #define PLANNER_UTILS_EXPORT __attribute__ ((visibility("default")))
  #define PLANNER_UTILS_IMPORT
  #if __GNUC__ >= 4
    #define PLANNER_UTILS_PUBLIC __attribute__ ((visibility("default")))
    #define PLANNER_UTILS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PLANNER_UTILS_PUBLIC
    #define PLANNER_UTILS_LOCAL
  #endif
  #define PLANNER_UTILS_PUBLIC_TYPE
#endif

#endif  // PLANNER_UTILS__VISIBILITY_CONTROL_H_
