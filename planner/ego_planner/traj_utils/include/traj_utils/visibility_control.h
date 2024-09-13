#ifndef TRAJ_UTILS__VISIBILITY_CONTROL_H_
#define TRAJ_UTILS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TRAJ_UTILS_EXPORT __attribute__ ((dllexport))
    #define TRAJ_UTILS_IMPORT __attribute__ ((dllimport))
  #else
    #define TRAJ_UTILS_EXPORT __declspec(dllexport)
    #define TRAJ_UTILS_IMPORT __declspec(dllimport)
  #endif
  #ifdef TRAJ_UTILS_BUILDING_LIBRARY
    #define TRAJ_UTILS_PUBLIC TRAJ_UTILS_EXPORT
  #else
    #define TRAJ_UTILS_PUBLIC TRAJ_UTILS_IMPORT
  #endif
  #define TRAJ_UTILS_PUBLIC_TYPE TRAJ_UTILS_PUBLIC
  #define TRAJ_UTILS_LOCAL
#else
  #define TRAJ_UTILS_EXPORT __attribute__ ((visibility("default")))
  #define TRAJ_UTILS_IMPORT
  #if __GNUC__ >= 4
    #define TRAJ_UTILS_PUBLIC __attribute__ ((visibility("default")))
    #define TRAJ_UTILS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TRAJ_UTILS_PUBLIC
    #define TRAJ_UTILS_LOCAL
  #endif
  #define TRAJ_UTILS_PUBLIC_TYPE
#endif

#endif  // TRAJ_UTILS__VISIBILITY_CONTROL_H_
