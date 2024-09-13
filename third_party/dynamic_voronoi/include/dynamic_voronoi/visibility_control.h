#ifndef DYNAMIC_VORONOI__VISIBILITY_CONTROL_H_
#define DYNAMIC_VORONOI__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DYNAMIC_VORONOI_EXPORT __attribute__ ((dllexport))
    #define DYNAMIC_VORONOI_IMPORT __attribute__ ((dllimport))
  #else
    #define DYNAMIC_VORONOI_EXPORT __declspec(dllexport)
    #define DYNAMIC_VORONOI_IMPORT __declspec(dllimport)
  #endif
  #ifdef DYNAMIC_VORONOI_BUILDING_LIBRARY
    #define DYNAMIC_VORONOI_PUBLIC DYNAMIC_VORONOI_EXPORT
  #else
    #define DYNAMIC_VORONOI_PUBLIC DYNAMIC_VORONOI_IMPORT
  #endif
  #define DYNAMIC_VORONOI_PUBLIC_TYPE DYNAMIC_VORONOI_PUBLIC
  #define DYNAMIC_VORONOI_LOCAL
#else
  #define DYNAMIC_VORONOI_EXPORT __attribute__ ((visibility("default")))
  #define DYNAMIC_VORONOI_IMPORT
  #if __GNUC__ >= 4
    #define DYNAMIC_VORONOI_PUBLIC __attribute__ ((visibility("default")))
    #define DYNAMIC_VORONOI_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DYNAMIC_VORONOI_PUBLIC
    #define DYNAMIC_VORONOI_LOCAL
  #endif
  #define DYNAMIC_VORONOI_PUBLIC_TYPE
#endif

#endif  // DYNAMIC_VORONOI__VISIBILITY_CONTROL_H_
