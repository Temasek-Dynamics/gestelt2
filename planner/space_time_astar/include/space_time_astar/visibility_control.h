#ifndef SPACE_TIME_ASTAR__VISIBILITY_CONTROL_H_
#define SPACE_TIME_ASTAR__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SPACE_TIME_ASTAR_EXPORT __attribute__ ((dllexport))
    #define SPACE_TIME_ASTAR_IMPORT __attribute__ ((dllimport))
  #else
    #define SPACE_TIME_ASTAR_EXPORT __declspec(dllexport)
    #define SPACE_TIME_ASTAR_IMPORT __declspec(dllimport)
  #endif
  #ifdef SPACE_TIME_ASTAR_BUILDING_LIBRARY
    #define SPACE_TIME_ASTAR_PUBLIC SPACE_TIME_ASTAR_EXPORT
  #else
    #define SPACE_TIME_ASTAR_PUBLIC SPACE_TIME_ASTAR_IMPORT
  #endif
  #define SPACE_TIME_ASTAR_PUBLIC_TYPE SPACE_TIME_ASTAR_PUBLIC
  #define SPACE_TIME_ASTAR_LOCAL
#else
  #define SPACE_TIME_ASTAR_EXPORT __attribute__ ((visibility("default")))
  #define SPACE_TIME_ASTAR_IMPORT
  #if __GNUC__ >= 4
    #define SPACE_TIME_ASTAR_PUBLIC __attribute__ ((visibility("default")))
    #define SPACE_TIME_ASTAR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SPACE_TIME_ASTAR_PUBLIC
    #define SPACE_TIME_ASTAR_LOCAL
  #endif
  #define SPACE_TIME_ASTAR_PUBLIC_TYPE
#endif

#endif  // SPACE_TIME_ASTAR__VISIBILITY_CONTROL_H_
