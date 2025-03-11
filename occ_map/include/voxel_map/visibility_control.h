#ifndef VOXEL_MAP__VISIBILITY_CONTROL_H_
#define VOXEL_MAP__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VOXEL_MAP_EXPORT __attribute__ ((dllexport))
    #define VOXEL_MAP_IMPORT __attribute__ ((dllimport))
  #else
    #define VOXEL_MAP_EXPORT __declspec(dllexport)
    #define VOXEL_MAP_IMPORT __declspec(dllimport)
  #endif
  #ifdef VOXEL_MAP_BUILDING_LIBRARY
    #define VOXEL_MAP_PUBLIC VOXEL_MAP_EXPORT
  #else
    #define VOXEL_MAP_PUBLIC VOXEL_MAP_IMPORT
  #endif
  #define VOXEL_MAP_PUBLIC_TYPE VOXEL_MAP_PUBLIC
  #define VOXEL_MAP_LOCAL
#else
  #define VOXEL_MAP_EXPORT __attribute__ ((visibility("default")))
  #define VOXEL_MAP_IMPORT
  #if __GNUC__ >= 4
    #define VOXEL_MAP_PUBLIC __attribute__ ((visibility("default")))
    #define VOXEL_MAP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VOXEL_MAP_PUBLIC
    #define VOXEL_MAP_LOCAL
  #endif
  #define VOXEL_MAP_PUBLIC_TYPE
#endif

#endif  // VOXEL_MAP__VISIBILITY_CONTROL_H_
