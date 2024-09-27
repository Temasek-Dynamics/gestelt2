#ifndef TRAJ_OPT__VISIBILITY_CONTROL_H_
#define TRAJ_OPT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TRAJ_OPT_EXPORT __attribute__ ((dllexport))
    #define TRAJ_OPT_IMPORT __attribute__ ((dllimport))
  #else
    #define TRAJ_OPT_EXPORT __declspec(dllexport)
    #define TRAJ_OPT_IMPORT __declspec(dllimport)
  #endif
  #ifdef TRAJ_OPT_BUILDING_LIBRARY
    #define TRAJ_OPT_PUBLIC TRAJ_OPT_EXPORT
  #else
    #define TRAJ_OPT_PUBLIC TRAJ_OPT_IMPORT
  #endif
  #define TRAJ_OPT_PUBLIC_TYPE TRAJ_OPT_PUBLIC
  #define TRAJ_OPT_LOCAL
#else
  #define TRAJ_OPT_EXPORT __attribute__ ((visibility("default")))
  #define TRAJ_OPT_IMPORT
  #if __GNUC__ >= 4
    #define TRAJ_OPT_PUBLIC __attribute__ ((visibility("default")))
    #define TRAJ_OPT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TRAJ_OPT_PUBLIC
    #define TRAJ_OPT_LOCAL
  #endif
  #define TRAJ_OPT_PUBLIC_TYPE
#endif

#endif  // TRAJ_OPT__VISIBILITY_CONTROL_H_
